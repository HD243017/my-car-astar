# my-car-astar: TurtleBot3 Waffle Pi A* 주행 및 비상정지 노드

TurtleBot3 Waffle Pi의 센서(LiDAR, 카메라) 및 AMCL Localization을 활용하여 목적지까지 A* 알고리즘 기반으로 자율주행하고, 카메라 객체 인식(YOLOv8) 시 비상 정지(Emergency Stop)를 수행하는 패키지입니다.

---

## 1. 사전 의존 패키지 설치 (Prerequisites)

PC 및 TurtleBot3에 다음 패키지와 라이브러리가 설치되어 있어야 합니다.

### ROS 2 기본 패키지 설치
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-bringup \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-v4l2-camera \
  ros-humble-image-transport-plugins

### Python 라이브러리 설치 (YOLOv8 및 영상 처리)
pip3 install ultralytics opencv-python

---

## 2. 빌드 및 환경 설정 (PC)

저장소를 클론한 후 워크스페이스 루트에서 빌드를 진행합니다.

cd ~/my_bot
colcon build --symlink-install
source install/setup.bash

참고: tb3_localization.launch.py 내의 맵 파일 기본 경로가 /home/student/my_bot/my_map2.yaml로 지정되어 있습니다. 경로가 다를 경우 런치 인자(map:=<맵경로>)로 전달하거나 경로를 맞춰주어야 합니다.

---

## 3. 실행 방법 (Execution Steps)

### Step 1. 터틀봇(SBC) 상에서 실행
터틀봇 기기 내부 터미널(SSH)에서 브링업 및 카메라 노드를 구동합니다.
(※ 연산 속도 및 레이턴시 최소화를 위해 카메라 해상도를 320x240으로 지정하여 실행합니다.)

- 터미널 1: TurtleBot3 로봇 브링업
ros2 launch turtlebot3_bringup robot.launch.py

- 터미널 2: V4L2 카메라 노드 구동
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[320,240]"

---

### Step 2. 원격 PC (Remote PC) 상에서 실행

- 터미널 1: AMCL Localization & Map Server & RViz2 실행
cd ~/my_bot
source install/setup.bash
ros2 launch my_bot tb3_localization.launch.py

1. RViz 창이 열리면 상단 툴바의 '2D Pose Estimate'를 클릭하여 로봇의 현재 초기 위치를 지정합니다.
2. 로봇 위치가 잡힌 후, 새 터미널에서 A* 주행 및 비상정지 노드를 실행합니다.

- 터미널 2: A* 주행 및 비상정지 제어 노드 실행
cd ~/my_bot
source install/setup.bash
ros2 run my_bot astar_obstacle_cam

3. RViz 상단 툴바의 '2D Goal Pose'를 이용해 목표 위치를 지정하면 경로 생성 후 자율 주행을 시작합니다.

---

## 4. 주요 기능 및 비상정지 구현 로직

1. A* 알고리즘 경로 계획 및 추종:
   - /map 토픽 수신 시 팽창(Inflation) 맵을 생성하여 안전 반경 확보.
   - 목적지 지정 시 A*를 이용해 충돌 없는 최적 경로 산출 후 /planned_path로 시각화.
   - Pure Pursuit 방식을 응용하여 산출된 Waypoint 추종 주행.

2. YOLOv8 기반 특정 객체 비상 정지 (Emergency Stop):
   - /image_raw/compressed 토픽을 실시간 수신하여 가중치 모델(weights/best.pt)로 추론.
   - 특정 타겟 클래스(turtle)가 일정 신뢰도(Confidence >= 0.70) 및 크기(Area >= 5000) 이상으로 전방에 검출되면 /cmd_vel을 즉시 영속도(0.0)로 퍼블리시하여 비상 정지.
   - 모니터링 창(cv2.imshow) 상단에 STATUS: EMERGENCY STOP 경고 표시.
   - 전방 장애물 제거 시 자동으로 비상 정지 해제 및 주행 재개.

3. LiDAR 전방 안전 정지:
   - 라이다(/scan) 전방 40도 범위 내에 0.20m 이내로 장애물이 접근할 경우 긴급 정지.