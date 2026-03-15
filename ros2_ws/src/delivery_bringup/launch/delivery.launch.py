"""配送系统节点启动脚本

启动 delivery_manager 配送管理节点。

使用：
  ros2 launch delivery_bringup delivery.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 站点配置文件路径
    default_station_config = PathJoinSubstitution(
        [FindPackageShare("delivery_bringup"), "config", "stations.yaml"]
    )

    # Launch 参数声明
    station_config_arg = DeclareLaunchArgument(
        "station_config",
        default_value=default_station_config,
        description="Path to station configuration YAML",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    publish_initial_pose_arg = DeclareLaunchArgument(
        "publish_initial_pose",
        default_value="true",
        description="Publish initial pose to AMCL",
    )

    initial_x_arg = DeclareLaunchArgument("initial_x", default_value="0.0")
    initial_y_arg = DeclareLaunchArgument("initial_y", default_value="0.0")
    initial_yaw_arg = DeclareLaunchArgument("initial_yaw", default_value="0.0")

    # delivery_manager 节点
    delivery_manager = Node(
        package="delivery_core",
        executable="delivery_manager",
        name="delivery_manager",
        output="screen",
        parameters=[
            {
                "station_config": LaunchConfiguration("station_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_initial_pose": LaunchConfiguration("publish_initial_pose"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "navigation_timeout_sec": 120.0,
                "wait_confirmation_timeout_sec": 60.0,
            }
        ],
    )

    return LaunchDescription(
        [
            station_config_arg,
            use_sim_time_arg,
            publish_initial_pose_arg,
            initial_x_arg,
            initial_y_arg,
            initial_yaw_arg,
            delivery_manager,
        ]
    )
