"""Gazebo 仿真环境启动脚本

启动 TurtleBot3 机器人 + 仿真环境（使用 TurtleBot3 标准世界）。

使用：
  export TURTLEBOT3_MODEL=waffle_pi
  ros2 launch delivery_bringup simulation.launch.py

注意：自定义世界（如 warehouse.sdf）需要额外的 gz_sim + robot spawn 配置，
当前版本尚未实现，后续版本会支持 world 参数切换。
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

    当前使用 turtlebot3_gazebo 标准世界。
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
