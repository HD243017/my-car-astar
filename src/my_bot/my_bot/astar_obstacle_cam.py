import os
import cv2
import numpy as np
import heapq
from math import atan2, sqrt, sin, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, CompressedImage

def inflate_map(grid_map, radius_cells=5):
    inflated = np.copy(grid_map)
    # 벽(>50) 이거나 미탐색(-1)인 영역 좌표 추출
    obs_y, obs_x = np.where((grid_map > 50) | (grid_map == -1))
    height, width = grid_map.shape

    # 벽 주변 radius_cells 범위만큼 벽(100)으로 확장
    for y, x in zip(obs_y, obs_x):
        y_min = max(0, y - radius_cells)
        y_max = min(height, y + radius_cells + 1)
        x_min = max(0, x - radius_cells)
        x_max = min(width, x + radius_cells + 1)
        inflated[y_min:y_max, x_min:x_max] = 100

    return inflated

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

def run_astar(map_data, start, end, width, height, logger=None):
    if not (0 <= start[0] < height and 0 <= start[1] < width):
        if logger: logger.error(f"[A* FAIL] Start {start} out of bounds")
        return None
    if not (0 <= end[0] < height and 0 <= end[1] < width):
        if logger: logger.error(f"[A* FAIL] End {end} out of bounds")
        return None

    # 오직 0(비어있는 영역)만 통과 가능 (-1 미탐색, 100 장애물 모두 차단)
    if map_data[start[0]][start[1]] != 0:
        if logger: logger.error(f"[A* FAIL] Start cell {start} is occupied ({map_data[start[0]][start[1]]})")
        return None
    if map_data[end[0]][end[1]] != 0:
        if logger: logger.error(f"[A* FAIL] End cell {end} is occupied ({map_data[end[0]][end[1]]})")
        return None

    start_node = NodeAStar(None, start)
    end_node = NodeAStar(None, end)
    
    open_list = []
    heapq.heappush(open_list, start_node)
    
    closed_set = set()
    open_dict = {start: start_node.g}
    
    moves = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
    loop_count = 0
    max_loops = 15000

    while open_list and loop_count < max_loops:
        loop_count += 1
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)

        if current_node.position == end_node.position:
            path = []
            curr = current_node
            while curr:
                path.append(curr.position)
                curr = curr.parent
            if logger:
                logger.info(f"[A* SUCCESS] Path found in {loop_count} steps! (Length: {len(path)})")
            return path[::-1]

        for move in moves:
            ny = current_node.position[0] + move[0]
            nx = current_node.position[1] + move[1]
            pos = (ny, nx)

            if not (0 <= ny < height and 0 <= nx < width):
                continue

            if map_data[ny][nx] != 0:
                continue

            if pos in closed_set:
                continue

            move_cost = 1.414 if (move[0] != 0 and move[1] != 0) else 1.0
            g_cost = current_node.g + move_cost

            if pos in open_dict and g_cost >= open_dict[pos]:
                continue

            new_node = NodeAStar(current_node, pos)
            new_node.g = g_cost
            new_node.h = sqrt((ny - end[0])**2 + (nx - end[1])**2)
            new_node.f = new_node.g + new_node.h

            open_dict[pos] = g_cost
            heapq.heappush(open_list, new_node)

    if logger:
        if loop_count >= max_loops:
            logger.error(f"[A* FAIL] Timed out! Exceeded {max_loops} iterations.")
        else:
            logger.error(f"[A* FAIL] Open list empty. No valid path exists.")
    return None

class AStarObstacleNavigator(Node):
    def __init__(self):
        super().__init__('astar_obstacle_cam')

        # 동시성 콜백 그룹 설정
        self.callback_group = ReentrantCallbackGroup()

        # YOLO 초기화
        try:
            model_path = os.path.join(get_package_share_directory('my_bot'), 'weights', 'best.pt')
        except Exception:
            model_path = os.path.expanduser('~/my_bot/src/my_bot/weights/best.pt')

        self.get_logger().info(f"Loading YOLO Model from: {model_path}")
        self.model = YOLO(model_path)
        self.target_class_name = 'turtle'

        # 판정 수치
        self.conf_threshold = 0.70  # 신뢰도
        self.min_box_area = 5000    # 최소면적
        self.emergency_stop_active = False

        # 프레임 밀림 방지 플래그
        self.is_inferencing = False

        # 주행 변수
        self.lookahead_distance = 0.5
        self.linear_velocity = 0.10
        self.obs_threshold = 0.20

        self.path = []
        self.current_waypoint_index = 0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.is_localized = False

        self.left_dist = 99.9
        self.front_dist = 99.9
        self.right_dist = 99.9

        self.map_data = None
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]
        self.goal_pose = [0.0, 0.0]

        # 퍼블리셔
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)  # RViz 경로 시각화 퍼블리셔

        # 섭스크라이버
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos,
            callback_group=self.callback_group
        )
        
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos,
            callback_group=self.callback_group
        )

        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10,
            callback_group=self.callback_group
        )

        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            image_qos,
            callback_group=self.callback_group
        )

        # 제어 루프 타이머
        self.timer = self.create_timer(0.1, self.control_loop, callback_group=self.callback_group)
        self.get_logger().info("Ready for Navigation! Set 2D Goal Pose in RViz.")

    def image_callback(self, msg):
        # 프레임 추론 중이면 현재 프레임은 Skip
        if self.is_inferencing:
            return

        self.is_inferencing = True

        try:
            # 압축 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                return

            # YOLOv8 추론
            results = self.model.predict(cv_image, imgsz=320, conf=self.conf_threshold, verbose=False)

            detected_emergency = False
            annotated_frame = cv_image.copy()

            # 검출 결과 바운딩 박스 표시
            annotated_frame = cv_image.copy()
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    confidence = float(box.conf[0])

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    box_width = x2 - x1
                    box_height = y2 - y1
                    box_area = box_width * box_height

                    if class_name == self.target_class_name:
                        # 0.7 이상 신뢰도 + 일정 크기 이상일 때 비상정지 조건 충족
                        if confidence >= self.conf_threshold and box_area >= self.min_box_area:
                            detected_emergency = True
                            color = (0, 0, 255)
                            status_txt = f"[EMERGENCY] {class_name} {confidence:.2f} ({box_width}x{box_height})"
                        else:
                            color = (0, 255, 255)
                            status_txt = f"{class_name} {confidence:.2f} ({box_width}x{box_height})"

                        # Bounding Box Label
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(
                            annotated_frame,
                            status_txt,
                            (x1, max(y1 - 10, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            color,
                            2
                        )

            # 검출 상태 알림
            if detected_emergency:
                if not self.emergency_stop_active:
                    self.get_logger().error(f"[EMERGENCY STOP] '{self.target_class_name}'")
                self.emergency_stop_active = True
            else:
                if self.emergency_stop_active:
                    self.get_logger().info("[EMERGENCY STOP RELEASE]")
                self.emergency_stop_active = False

            # 모니터 화면 상단에 현재 비상정지 상태 표시
            top_bar_color = (0, 0, 255) if self.emergency_stop_active else (0, 255, 0)
            top_bar_text = "STATUS: EMERGENCY STOP" if self.emergency_stop_active else "STATUS: DRIVING / READY"
            cv2.putText(annotated_frame, top_bar_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, top_bar_color, 2)

            cv2.imshow("TurtleBot3 YOLOv8 Monitor", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
        finally:
            self.is_inferencing = False

    def world_to_grid(self, world):
        return (
            int((world[1] - self.map_origin[1]) / self.map_resolution),
            int((world[0] - self.map_origin[0]) / self.map_resolution)
        )

    def grid_to_world(self, grid):
        return [
            (grid[1] * self.map_resolution) + self.map_origin[0],
            (grid[0] * self.map_resolution) + self.map_origin[1]
        ]

    def publish_path_viz(self):
        """계획된 경로를 RViz 시각화용 nav_msgs/Path 메시지로 퍼블리시"""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for pt in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub_path.publish(msg)

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

        raw_map = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        self.map_data = inflate_map(raw_map, radius_cells=5)
        self.get_logger().info(f"Map Received: {self.map_width} x {self.map_height}")

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.is_localized = True

    def goal_callback(self, msg):
        self.stop_robot()  # 새 목표 수신 시 즉각 정지

        self.goal_pose = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f"New Goal: {self.goal_pose}")

        if self.map_data is None or not self.is_localized:
            self.get_logger().warn("Map or Robot Pose not ready!")
            return

        # 함수 호출 방식으로 그리드 좌표 산출
        start = self.world_to_grid([self.current_x, self.current_y])
        end = self.world_to_grid(self.goal_pose)

        astar_path = run_astar(self.map_data, start, end, self.map_width, self.map_height, logger=self.get_logger())

        if astar_path is not None:
            # grid_to_world를 이용해 경로 복원
            self.path = [self.grid_to_world(pt) for pt in astar_path]
            self.current_waypoint_index = 0
            self.publish_path_viz()  # RViz로 계획 경로 퍼블리시
            self.get_logger().info(f"Path generated with {len(self.path)} points!")
        else:
            self.path = []
            self.current_waypoint_index = 0
            self.publish_path_viz()  # 빈 경로 퍼블리시하여 이전 경로 잔상 삭제
            self.get_logger().warn("Path not found!")

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:40] + msg.ranges[-40:]
        left_ranges = msg.ranges[20:70]
        right_ranges = msg.ranges[-70:-20]

        self.front_dist = self.get_min_dist(front_ranges)
        self.left_dist = self.get_min_dist(left_ranges)
        self.right_dist = self.get_min_dist(right_ranges)

    def get_min_dist(self, range_list):
        valid_values = [x for x in range_list if 0.05 < x < 10.0]
        if valid_values:
            return min(valid_values)
        return 99.9

    def control_loop(self):
        if self.emergency_stop_active:
            self.stop_robot()
            return

        if not self.is_localized or len(self.path) == 0:
            return

        # 라이다 전방 긴급 장애물 정지
        if self.front_dist < self.obs_threshold:
            self.get_logger().warn(f"Obstacle Ahead! Stopping... ({self.front_dist:.2f}m)")
            self.stop_robot()
            return

        self.follow_path()

    def follow_path(self):
        if not self.path or self.current_waypoint_index >= len(self.path):
            self.stop_robot()
            self.path = []
            self.publish_path_viz()
            return

        # 최종 목적지 도착 판정
        final_goal = self.path[-1]
        dist_to_final = sqrt((final_goal[0] - self.current_x)**2 + (final_goal[1] - self.current_y)**2)
        if dist_to_final < 0.15:
            self.get_logger().info("목표 지점에 도착했습니다!")
            self.stop_robot()
            self.path = []
            self.publish_path_viz()
            return

        # Pure Pursuit Lookahead Point 선정
        target_point = self.path[-1]
        for i in range(self.current_waypoint_index, len(self.path)):
            pt = self.path[i]
            d = sqrt((pt[0] - self.current_x)**2 + (pt[1] - self.current_y)**2)
            if d >= self.lookahead_distance:
                target_point = pt
                self.current_waypoint_index = i
                break

        dx = target_point[0] - self.current_x
        dy = target_point[1] - self.current_y
        distance = sqrt(dx**2 + dy**2)

        target_angle = atan2(dy, dx)
        alpha = target_angle - self.current_yaw
        while alpha > pi: alpha -= 2 * pi
        while alpha < -pi: alpha += 2 * pi

        cmd = Twist()

        # 헤딩 오차가 45도 이상이면 제자리 회전
        if abs(alpha) > 0.8:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if alpha > 0 else -0.5
        else:
            cmd.linear.x = self.linear_velocity
            cmd.angular.z = self.linear_velocity * (2.0 * sin(alpha)) / max(distance, 0.05)
            cmd.angular.z = max(-0.8, min(0.8, cmd.angular.z))

        self.publisher_.publish(cmd)
        
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AStarObstacleNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



