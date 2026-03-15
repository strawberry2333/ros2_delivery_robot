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
    """生成 Gazebo 仿真环境的启动描述

    本函数完成以下工作：
    1. 读取 TURTLEBOT3_MODEL 环境变量，确定机器人型号
    2. 设置环境变量确保子进程继承正确的机器人型号
    3. 调用 turtlebot3_gazebo 提供的标准世界启动文件

    turtlebot3_world 场景包含一个带有障碍物的室内环境，
    适合测试导航和避障功能。
    """

    # 是否使用仿真时间，仿真环境下应始终为 true
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # 从环境变量获取 TurtleBot3 机器人型号
    # 支持的型号：burger, waffle, waffle_pi
    # 本项目使用 waffle_pi（带深度相机和激光雷达）
    turtlebot3_model = os.environ.get("TURTLEBOT3_MODEL", "waffle_pi")

    # 获取 turtlebot3_gazebo 包的安装路径，用于定位世界启动文件
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    # 包含 TurtleBot3 World 仿真场景的启动文件
    # 该启动文件内部会：
    #   - 启动 Gazebo 仿真器（ros_gz_sim）
    #   - 加载 TurtleBot3 World 地图模型
    #   - 生成 TurtleBot3 机器人实体
    #   - 启动 robot_state_publisher 发布机器人 TF
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            # 设置 TURTLEBOT3_MODEL 环境变量，确保所有子进程使用一致的机器人型号
            # 这是 TurtleBot3 生态系统的标准做法，许多包依赖此变量加载对应的 URDF 和配置
            SetEnvironmentVariable("TURTLEBOT3_MODEL", turtlebot3_model),
            # 声明 use_sim_time 启动参数
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            # 启动 Gazebo 仿真世界
            world_launch,
        ]
    )
