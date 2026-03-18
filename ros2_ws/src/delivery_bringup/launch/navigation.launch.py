"""Nav2 导航栈启动脚本。

默认加载仓库地图，并使用轻量级 RViz 配置。RViz 启动被延后，
以减少初始定位阶段的时间戳/队列告警。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Nav2 导航栈的启动描述。"""

    turtlebot3_nav_dir = get_package_share_directory("turtlebot3_navigation2")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    delivery_bringup_dir = get_package_share_directory("delivery_bringup")
    delivery_simulation_dir = get_package_share_directory("delivery_simulation")

    turtlebot3_model = os.environ.get("TURTLEBOT3_MODEL", "waffle_pi")
    ros_distro = os.environ.get("ROS_DISTRO", "")

    param_file_name = f"{turtlebot3_model}.yaml"
    if ros_distro == "humble":
        default_params_file = os.path.join(
            turtlebot3_nav_dir, "param", ros_distro, param_file_name
        )
    else:
        default_params_file = os.path.join(
            turtlebot3_nav_dir, "param", param_file_name
        )

    default_map = os.path.join(delivery_simulation_dir, "maps", "warehouse.yaml")
    default_rviz_config = os.path.join(
        delivery_bringup_dir, "rviz", "warehouse_navigation.rviz"
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
            "autostart": "true",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": LaunchConfiguration("rviz_software_rendering")
        },
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    rviz_launch = TimerAction(
        period=LaunchConfiguration("rviz_delay"),
        actions=[rviz_node],
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
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Full path to the Nav2 parameter file",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Whether to launch RViz",
            ),
            DeclareLaunchArgument(
                "rviz_delay",
                default_value="15.0",
                description="Delay RViz startup until Nav2 and TF are ready",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Full path to the RViz config file",
            ),
            DeclareLaunchArgument(
                "rviz_software_rendering",
                default_value="1",
                description="Set LIBGL_ALWAYS_SOFTWARE for RViz (1=true, 0=false)",
            ),
            nav2_launch,
            rviz_launch,
        ]
    )
