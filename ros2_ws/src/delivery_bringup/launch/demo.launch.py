"""一键启动完整配送系统。

这个 launch 是对外的 demo 入口，负责把仿真、Nav2 和配送业务节点串起来。
默认启动仓库仿真、仓库地图和配送节点。
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

    这些延迟不是业务逻辑，而是工程上的启动缓冲。
    目标是减少“系统已启动但依赖还没 ready”的启动抖动。

    时序设计说明：
    - 5 秒延迟：Gazebo 需要加载世界模型、生成机器人、启动传感器插件
    - 15 秒延迟：Nav2 需要完成 AMCL 定位初始化、代价地图构建、规划器就绪
    - 如果在性能较低的机器上运行，可能需要增大延迟值
    """

    # 获取 delivery_bringup 包的安装路径，用于定位子 launch 文件。
    # 这样 demo 可以稳定引用同包内的 simulation/navigation/delivery launch。
    bringup_dir = get_package_share_directory("delivery_bringup")

    # 是否使用仿真时间。
    # 仿真环境下必须为 true，以确保所有节点使用 Gazebo 发布的 /clock 时间。
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")
    rviz = LaunchConfiguration("rviz")
    rviz_delay = LaunchConfiguration("rviz_delay")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_software_rendering = LaunchConfiguration("rviz_software_rendering")
    default_world = os.path.join(
        get_package_share_directory("delivery_simulation"), "worlds", "warehouse.sdf"
    )
    default_map = os.path.join(
        get_package_share_directory("delivery_simulation"), "maps", "warehouse.yaml"
    )

    # 第 1 阶段：仿真环境。
    # 这里先启动 world、机器人和桥接，再把后续节点挂到 TimerAction 里。
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "yaw": yaw,
        }.items(),
    )

    # 第 2 阶段：导航栈。
    # 延迟启动是为了给 Gazebo、robot_state_publisher 和传感器桥接留出时间。
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "navigation.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_yaml,
                    "rviz": rviz,
                    "rviz_delay": rviz_delay,
                    "rviz_config": rviz_config,
                    "rviz_software_rendering": rviz_software_rendering,
                }.items(),
            )
        ],
    )

    # 第 3 阶段：配送业务节点。
    # 需要等待 Nav2 与定位初始化，否则 manager 会在就绪检查阶段超时。
    delivery_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "delivery.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "initial_x": x_pose,
                    "initial_y": y_pose,
                    "initial_yaw": yaw,
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            # 声明全局启动参数，允许用户通过命令行覆盖默认值。
            # 例如：ros2 launch delivery_bringup demo.launch.py use_sim_time:=false
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Full path to the Gazebo world file",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to the Nav2 map yaml",
            ),
            DeclareLaunchArgument(
                "x_pose",
                default_value="-2.0",
                description="Initial robot X pose",
            ),
            DeclareLaunchArgument(
                "y_pose",
                default_value="-0.5",
                description="Initial robot Y pose",
            ),
            DeclareLaunchArgument(
                "yaw",
                default_value="0.0",
                description="Initial robot yaw",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Whether to launch RViz",
            ),
            DeclareLaunchArgument(
                "rviz_delay",
                default_value="15.0",
                description="Delay RViz startup until navigation is ready",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    bringup_dir, "rviz", "warehouse_navigation.rviz"
                ),
                description="Full path to the RViz config file",
            ),
            DeclareLaunchArgument(
                "rviz_software_rendering",
                default_value="1",
                description="Set LIBGL_ALWAYS_SOFTWARE for RViz (1=true, 0=false)",
            ),
            # 先仿真，再导航，最后配送节点。
            simulation_launch,
            navigation_launch,
            delivery_launch,
        ]
    )
