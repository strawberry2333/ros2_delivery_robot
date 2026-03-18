"""Gazebo 仿真环境启动脚本。

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

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")

    turtlebot3_model = os.environ.get("TURTLEBOT3_MODEL", "waffle_pi")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    delivery_simulation_dir = get_package_share_directory("delivery_simulation")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    turtlebot3_launch_dir = os.path.join(turtlebot3_gazebo_dir, "launch")

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

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v2 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-g -v2 ",
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_launch_dir, "robot_state_publisher.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

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

    image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
    )

    actions = [
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
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        bridge_cmd,
    ]
    if turtlebot3_model != "burger":
        actions.append(image_bridge_cmd)

    return LaunchDescription(actions)
