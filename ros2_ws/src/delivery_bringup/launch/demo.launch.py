"""一键启动完整配送系统

启动顺序：Gazebo 仿真 → Nav2 导航栈 → 配送管理节点

使用：
  export TURTLEBOT3_MODEL=waffle_pi
  ros2 launch delivery_bringup demo.launch.py

验证：
  ros2 topic echo /delivery_status
  ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
    "{order: {order_id: 'order_001', pickup_station: 'station_A', dropoff_station: 'station_C', priority: 0}}"
  ros2 service call /confirm_load std_srvs/srv/Trigger
  ros2 service call /confirm_unload std_srvs/srv/Trigger
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory("delivery_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # --- 阶段 1: Gazebo 仿真环境 ---
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # --- 阶段 2: Nav2 导航栈 (延迟 5 秒等待 Gazebo 就绪) ---
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "navigation.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    # --- 阶段 3: 配送管理节点 (延迟 15 秒等待 Nav2 就绪) ---
    delivery_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "delivery.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            simulation_launch,
            navigation_launch,
            delivery_launch,
        ]
    )
