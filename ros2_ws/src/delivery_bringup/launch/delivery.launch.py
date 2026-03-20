"""配送系统节点启动脚本。

这个 launch 只负责把“配送业务链路”拉起来，不负责仿真和 Nav2 本身。
默认使用仓库场景的站点配置和初始位姿。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成配送系统的启动描述

    启动顺序：
    1. delivery_executor (LifecycleNode) — BT 宿主节点
    2. delivery_lifecycle_manager — 自动 configure + activate executor
    3. delivery_manager — 订单管理调度节点

    这个顺序是有意设计的：
    - 先起 executor，让生命周期节点实例先存在
    - 再由 lifecycle manager 把 executor 推到 Active
    - 最后启动 manager 接单，避免“接到单但执行端还没就绪”
    """

    bringup_share = FindPackageShare("delivery_bringup")
    core_share = FindPackageShare("delivery_core")

    # 默认配置文件路径都来自包自身的 share 目录。
    # launch 参数可覆盖这些默认值，方便在不同地图或树文件间切换。
    default_station_config = PathJoinSubstitution(
        [bringup_share, "config", "stations.yaml"]
    )
    default_params_file = PathJoinSubstitution(
        [bringup_share, "config", "delivery_manager.yaml"]
    )
    default_lifecycle_params = PathJoinSubstitution(
        [bringup_share, "config", "lifecycle_manager.yaml"]
    )
    default_tree_file = PathJoinSubstitution(
        [core_share, "behavior_trees", "single_delivery_robust.xml"]
    )

    # --- 启动参数声明 ---
    # 这些参数会继续传递给下游节点，不在本 launch 中直接消费。
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
    initial_x_arg = DeclareLaunchArgument("initial_x", default_value="-2.0")
    initial_y_arg = DeclareLaunchArgument("initial_y", default_value="-0.5")
    initial_yaw_arg = DeclareLaunchArgument("initial_yaw", default_value="0.0")
    tree_file_arg = DeclareLaunchArgument(
        "tree_file",
        default_value=default_tree_file,
        description="Path to BehaviorTree XML file",
    )

    # --- delivery_executor 节点（LifecycleNode，启动后处于 Unconfigured） ---
    # 它只提供单订单执行能力，不负责队列和用户接单。
    delivery_executor = Node(
        package="delivery_core",
        executable="delivery_executor",
        name="delivery_executor",
        output="screen",
        parameters=[
            {
                "station_config": LaunchConfiguration("station_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "tree_file": LaunchConfiguration("tree_file"),
            },
        ],
    )

    # --- lifecycle_manager 节点（自动 configure + activate executor） ---
    # 它是启动编排器，确保 executor 在 manager 接单前已经可执行。
    lifecycle_manager = Node(
        package="delivery_lifecycle",
        executable="delivery_lifecycle_manager",
        name="delivery_lifecycle_manager",
        output="screen",
        parameters=[
            default_lifecycle_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # --- delivery_manager 节点（订单管理调度） ---
    # 它是对外服务入口，负责 submit/cancel/report，并把任务交给 executor。
    delivery_manager = Node(
        package="delivery_core",
        executable="delivery_manager",
        name="delivery_manager",
        output="screen",
        parameters=[
            default_params_file,
            {
                "station_config": LaunchConfiguration("station_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_initial_pose": LaunchConfiguration("publish_initial_pose"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
            },
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
            tree_file_arg,
            # 启动顺序：executor → lifecycle_manager → manager。
            # 这里不做额外 Timer 延迟，因为生命周期管理器和 executor 自身会等待依赖就绪。
            delivery_executor,
            lifecycle_manager,
            delivery_manager,
        ]
    )
