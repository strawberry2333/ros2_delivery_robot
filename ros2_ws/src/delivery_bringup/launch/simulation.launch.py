"""Gazebo 仿真环境启动脚本。

这个 launch 只负责仿真侧：Gazebo、机器人模型、桥接和 robot_state_publisher。
默认加载仓库世界，并在仓库默认出生点生成 TurtleBot3。
"""

import os

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Gazebo 仿真环境的启动描述。"""

    # 仿真相关输入参数，主要来自 demo.launch.py 的透传。
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")

    # 这些包提供机器人模型、Gazebo 资源和 ros_gz 桥接。
    turtlebot3_model = os.environ.get("TURTLEBOT3_MODEL", "waffle_pi")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    delivery_simulation_dir = get_package_share_directory("delivery_simulation")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    turtlebot3_launch_dir = os.path.join(turtlebot3_gazebo_dir, "launch")

    # model_sdf 和 bridge_params 必须与 TurtleBot3 模型保持一致。
    model_folder = f"turtlebot3_{turtlebot3_model}"
    model_sdf = os.path.join(
        turtlebot3_gazebo_dir, "models", model_folder, "model.sdf"
    )
    bridge_params = os.path.join(
        turtlebot3_gazebo_dir, "params", f"{model_folder}_bridge.yaml"
    )
    default_world = os.path.join(
        delivery_simulation_dir, "worlds", "warehouse.sdf"
    )

    # 后端 Gazebo server。
    # 读取 world 参数决定加载哪个 SDF 场景。
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v2 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # 前端 Gazebo client，仅用于可视化。
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-g -v2 ",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # robot_state_publisher 负责发布 URDF 相关 TF，供 Nav2 和 RViz 使用。
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_launch_dir, "robot_state_publisher.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 在 world 中生成 TurtleBot3 实体。
    # spawn 的初始位姿要和 delivery.launch.py 里的初始位姿保持一致。
    spawn_turtlebot_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            turtlebot3_model,
            "-file",
            model_sdf,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.01",
            "-Y",
            yaw,
        ],
        output="screen",
    )

    # ros_gz_bridge 负责把仿真话题桥接到 ROS 2。
    bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    # 非 burger 模型通常带摄像头，因此再加一条图像桥接。
    image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
    )

    actions = [
        # 先设置环境变量，再声明参数，再启动仿真主体。
        SetEnvironmentVariable("TURTLEBOT3_MODEL", turtlebot3_model),
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
            "x_pose",
            default_value="-2.0",
            description="Initial robot X pose in the simulation world",
        ),
        DeclareLaunchArgument(
            "y_pose",
            default_value="-0.5",
            description="Initial robot Y pose in the simulation world",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Initial robot yaw in the simulation world",
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.path.join(turtlebot3_gazebo_dir, "models"),
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.path.join(delivery_simulation_dir, "models"),
        ),
        # Gazebo server/client、TF、实体生成和桥接按这个顺序启动。
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        bridge_cmd,
    ]
    if turtlebot3_model != "burger":
        actions.append(image_bridge_cmd)

    return LaunchDescription(actions)
