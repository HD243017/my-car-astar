import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 실제 로봇 시간이므로 false 고정
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # 1. slam_toolbox 실행
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    # 2. RViz2 시각화 실행
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_cartographer'), 
        'rviz', 'turtlebot3_cartographer.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        slam_launch,
        rviz_node
    ])