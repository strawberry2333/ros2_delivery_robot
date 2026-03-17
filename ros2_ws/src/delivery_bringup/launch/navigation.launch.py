"""Nav2 导航栈启动脚本

包装 turtlebot3_navigation2 的标准 launch，自动使用 waffle_pi.yaml
（含 enable_stamped_cmd_vel: true）并启动 RViz。

使用：
  ros2 launch delivery_bringup navigation.launch.py
  # 使用仓库地图：
  ros2 launch delivery_bringup navigation.launch.py map:=/path/to/warehouse.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Nav2 导航栈的启动描述（基于 TurtleBot3 标准导航 launch）"""

    turtlebot3_nav_dir = get_package_share_directory("turtlebot3_navigation2")

    # 默认使用 TurtleBot3 标准地图
    default_map = os.path.join(turtlebot3_nav_dir, "map", "map.yaml")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_nav_dir, "launch", "navigation2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to map yaml file",
            ),
            nav2_launch,
        ]
    )
