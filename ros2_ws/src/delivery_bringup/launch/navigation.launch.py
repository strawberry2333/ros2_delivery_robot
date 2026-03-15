"""Nav2 导航栈启动脚本

启动完整的 Nav2 导航栈，使用 TurtleBot3 标准地图。

使用：
  ros2 launch delivery_bringup navigation.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Nav2 导航栈的启动描述

    本函数完成以下工作：
    1. 定位 Nav2 和 TurtleBot3 导航包的安装路径
    2. 加载 TurtleBot3 自带的地图文件（也可通过参数指定自定义地图）
    3. 调用 nav2_bringup 标准启动文件，启动完整导航栈

    Nav2 导航栈启动后包含以下核心组件：
    - AMCL：自适应蒙特卡洛定位，基于粒子滤波的定位算法
    - 全局代价地图 (global_costmap)：用于全局路径规划
    - 局部代价地图 (local_costmap)：用于局部避障
    - 路径规划器 (planner_server)：生成从起点到目标的全局路径
    - 控制器 (controller_server)：跟踪路径并执行局部避障
    - 行为服务器 (behavior_server)：处理异常情况（如原地旋转、后退）
    - BT 导航服务器 (bt_navigator)：管理导航行为树
    - 航点跟随器 (waypoint_follower)：支持多航点导航
    """

    # 是否使用仿真时间
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # 获取 nav2_bringup 包路径，包含 Nav2 的标准启动文件
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    # 获取 turtlebot3_navigation2 包路径，包含 TurtleBot3 专用的地图和导航配置
    turtlebot3_nav_dir = get_package_share_directory("turtlebot3_navigation2")

    # TurtleBot3 自带的地图文件路径
    # 该地图与 turtlebot3_world Gazebo 仿真场景对应
    # 用户可通过 map 参数指定自定义地图：
    #   ros2 launch delivery_bringup navigation.launch.py map:=/path/to/custom_map.yaml
    default_map = os.path.join(turtlebot3_nav_dir, "map", "map.yaml")

    # 包含 Nav2 标准 bringup 启动文件
    # bringup_launch.py 内部会启动所有 Nav2 核心节点，并加载默认参数
    # 传入地图路径和仿真时间配置
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": LaunchConfiguration("map"),
        }.items(),
    )

    return LaunchDescription(
        [
            # 声明 use_sim_time 参数
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),
            # 声明 map 参数，允许用户指定自定义地图文件
            # 默认使用 TurtleBot3 自带的 turtlebot3_world 地图
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to map yaml file",
            ),
            # 启动 Nav2 导航栈
            nav2_launch,
        ]
    )
