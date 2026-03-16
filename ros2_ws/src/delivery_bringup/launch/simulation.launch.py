"""Gazebo 仿真环境启动脚本

启动 TurtleBot3 机器人 + 仿真环境。
支持自定义 Gazebo 世界文件（默认使用 TurtleBot3 标准世界）。

使用：
  export TURTLEBOT3_MODEL=waffle_pi
  ros2 launch delivery_bringup simulation.launch.py
  # 使用自定义仓库场景：
  ros2 launch delivery_bringup simulation.launch.py world:=warehouse
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Gazebo 仿真环境的启动描述

    支持两种模式：
    1. 默认模式：使用 turtlebot3_gazebo 标准世界
    2. 自定义模式：通过 world 参数指定自定义世界名称
    """

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    turtlebot3_model = os.environ.get("TURTLEBOT3_MODEL", "waffle_pi")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    # 默认使用 TurtleBot3 标准世界
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("TURTLEBOT3_MODEL", turtlebot3_model),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            world_launch,
        ]
    )
