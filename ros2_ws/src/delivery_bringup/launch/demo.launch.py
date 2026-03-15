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
    """生成完整配送系统的启动描述

    本函数按照严格的时序分三个阶段启动所有子系统：
    1. 立即启动 Gazebo 仿真环境（加载世界模型和机器人）
    2. 延迟 5 秒后启动 Nav2 导航栈（等待 Gazebo 完成初始化、发布传感器数据）
    3. 延迟 15 秒后启动配送管理节点（等待 Nav2 完成地图加载和定位初始化）

    时序设计说明：
    - 5 秒延迟：Gazebo 需要加载世界模型、生成机器人、启动传感器插件
    - 15 秒延迟：Nav2 需要完成 AMCL 定位初始化、代价地图构建、规划器就绪
    - 如果在性能较低的机器上运行，可能需要增大延迟值
    """

    # 获取 delivery_bringup 包的安装路径，用于定位子 launch 文件
    bringup_dir = get_package_share_directory("delivery_bringup")

    # 是否使用仿真时间。仿真环境下必须为 true，以确保所有节点使用 Gazebo 发布的 /clock 时间
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # --- 阶段 1: Gazebo 仿真环境 ---
    # 立即启动，加载 TurtleBot3 World 场景和机器人模型
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # --- 阶段 2: Nav2 导航栈 (延迟 5 秒等待 Gazebo 就绪) ---
    # TimerAction 确保 Gazebo 完成初始化后再启动导航栈
    # 导航栈包括：AMCL 定位、全局/局部代价地图、路径规划器、控制器
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
    # TimerAction 确保 Nav2 完全就绪后再启动配送逻辑
    # delivery_manager 启动时会等待 TF 树和 /clock 话题可用
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
            # 声明 use_sim_time 参数，允许用户通过命令行覆盖默认值
            # 例如：ros2 launch delivery_bringup demo.launch.py use_sim_time:=false
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            # 按阶段顺序添加启动动作
            simulation_launch,
            navigation_launch,
            delivery_launch,
        ]
    )
