"""Gazebo 仿真环境启动脚本

启动 TurtleBot3 标准仿真环境 (Gazebo Harmonic)。
需要预先设置 TURTLEBOT3_MODEL 环境变量。

使用：
  export TURTLEBOT3_MODEL=waffle_pi
  ros2 launch delivery_bringup simulation.launch.py
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
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    turtlebot3_model = os.environ.get("TURTLEBOT3_MODEL", "waffle_pi")

    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
        ),
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
