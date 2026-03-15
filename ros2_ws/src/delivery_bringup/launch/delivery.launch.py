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
    """生成配送管理节点的启动描述

    本函数完成以下工作：
    1. 定位站点配置文件和节点参数文件
    2. 声明所有可配置的启动参数（站点配置、初始位姿等）
    3. 创建 delivery_manager 节点，加载参数文件并应用命令行覆盖值

    参数加载机制：
    - 首先加载 config/delivery_manager.yaml 中的默认参数
    - 然后用 launch 参数（命令行传入值）覆盖对应参数
    - 这种两级加载方式允许用户灵活配置而无需修改 YAML 文件
    """

    # 获取 delivery_bringup 包的 share 目录路径
    bringup_share = FindPackageShare("delivery_bringup")

    # 站点配置文件路径：定义所有配送站点的坐标和类型
    # 默认使用 config/stations.yaml，可通过 station_config 参数覆盖
    default_station_config = PathJoinSubstitution(
        [bringup_share, "config", "stations.yaml"]
    )

    # delivery_manager 节点参数文件路径：定义节点运行时的各项参数
    # 包含超时时间、坐标系名称、导航动作名称等配置
    default_params_file = PathJoinSubstitution(
        [bringup_share, "config", "delivery_manager.yaml"]
    )

    # --- 启动参数声明 ---

    # 站点配置文件路径参数
    # 用户可指定自定义站点配置：delivery.launch.py station_config:=/path/to/custom_stations.yaml
    station_config_arg = DeclareLaunchArgument(
        "station_config",
        default_value=default_station_config,
        description="Path to station configuration YAML",
    )

    # 是否使用仿真时间
    # 仿真环境下设为 true，真实机器人环境下设为 false
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    # 是否在启动时发布初始位姿到 AMCL
    # 设为 true 时，节点启动后会自动发布 /initialpose 消息
    # 用于在仿真环境中自动完成 AMCL 定位初始化，避免手动在 RViz 中设置
    publish_initial_pose_arg = DeclareLaunchArgument(
        "publish_initial_pose",
        default_value="true",
        description="Publish initial pose to AMCL",
    )

    # 机器人初始位姿参数（在地图坐标系 map frame 下）
    # initial_x: 初始 X 坐标（单位：米），默认 0.0
    initial_x_arg = DeclareLaunchArgument("initial_x", default_value="0.0")
    # initial_y: 初始 Y 坐标（单位：米），默认 0.0
    initial_y_arg = DeclareLaunchArgument("initial_y", default_value="0.0")
    # initial_yaw: 初始朝向角（单位：弧度），默认 0.0，范围 [-pi, pi]
    initial_yaw_arg = DeclareLaunchArgument("initial_yaw", default_value="0.0")

    # 获取 delivery_core 包的 share 目录路径（用于查找行为树 XML）
    core_share = FindPackageShare("delivery_core")

    # 行为树 XML 文件路径
    default_tree_file = PathJoinSubstitution(
        [core_share, "behavior_trees", "single_delivery.xml"]
    )

    # --- delivery_executor 节点配置（BT 宿主，必须在 manager 之前启动） ---
    delivery_executor = Node(
        package="delivery_core",
        executable="delivery_executor",
        name="delivery_executor",
        output="screen",
        parameters=[
            {
                "station_config": LaunchConfiguration("station_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "tree_file": default_tree_file,
            },
        ],
    )

    # --- delivery_manager 节点配置 ---
    # 节点参数加载顺序：
    #   1. 先加载 default_params_file（delivery_manager.yaml）中的全部参数
    #   2. 再用字典中的 LaunchConfiguration 值覆盖同名参数
    # 这样用户既可以修改 YAML 文件设置默认值，也可以通过命令行临时覆盖
    delivery_manager = Node(
        package="delivery_core",
        executable="delivery_manager",
        name="delivery_manager",
        output="screen",
        parameters=[
            # 第一层：从 YAML 文件加载基础参数
            default_params_file,
            # 第二层：用 launch 参数覆盖（优先级更高）
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
            # 先声明所有参数，再添加节点
            station_config_arg,
            use_sim_time_arg,
            publish_initial_pose_arg,
            initial_x_arg,
            initial_y_arg,
            initial_yaw_arg,
            # 先启动 executor（提供 Action Server），再启动 manager
            delivery_executor,
            delivery_manager,
        ]
    )
