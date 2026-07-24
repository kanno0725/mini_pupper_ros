#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Gazebo Harmonic (gz sim) 用の bringup。
# オリジナルの bringup.launch.py は Gazebo Classic (gazebo_ros) 前提のため、
# ROS 2 Jazzy + Gazebo Harmonic 環境向けに ros_gz ベースで再構成したもの。
#
# パイプライン:
#   /cmd_vel → twist_to_command_converter → stanford_controller（歩行生成）
#   → simple_quadruped_controller (ros2_control) → gz_ros2_control → Gazebo

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # robot_state_publisher（use_gazebo_hardware=true で gz_ros2_control 構成の URDF を生成）
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mini_pupper_description"),
                "launch", "mini_pupper_description.launch.py"])),
        launch_arguments={
            "use_sim_time": "true",
            "use_gazebo_hardware": "true",
        }.items()
    )

    # Gazebo Harmonic 起動。
    # GZ_SIM_SYSTEM_PLUGIN_PATH がないと gz_ros2_control プラグインが見つからない。
    # world は stock の empty.sdf ではなく、Imu / Sensors システムを足した自作 playground.sdf を使う
    # （stock empty.sdf にはこれらが無く、IMU センサや gpu_lidar が動かないため）。
    world_path = PathJoinSubstitution([
        FindPackageShare("mini_pupper_simulation"), "worlds", "playground.sdf"])
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        additional_env={"GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/ros/jazzy/lib"},
        output="screen",
    )

    # Gazebo の /clock を ROS に橋渡し（use_sim_time に必須）
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Gazebo の IMU（gz.msgs.IMU）を ROS の sensor_msgs/Imu に橋渡し。
    # URDF の imu センサが <topic>imu/data</topic> に publish する gz topic を、
    # stanford_controller_node が購読する ROS の /imu/data に変換する。
    # 「[」は gz→ROS の一方向を表す。
    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        output="screen",
    )

    # Gazebo の LiDAR（gz.msgs.LaserScan）を ROS の sensor_msgs/LaserScan に橋渡し。
    # URDF の gpu_lidar センサが <topic>scan</topic> に publish する gz topic を、
    # Phase 4 の回避ノードなどが購読する ROS の /scan に変換する。
    # 描画系センサなので world 側に Sensors システムのロードが必要（playground.sdf に追加済み）。
    # 「[」は gz→ROS の一方向を表す。
    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        output="screen",
    )

    # Gazebo の Odometry（gz.msgs.Odometry）を ROS の nav_msgs/Odometry に橋渡し。
    # URDF の OdometryPublisher システムが <odom_topic>odom</odom_topic> に publish する
    # gz topic を、odom_tf_broadcaster が購読する ROS の /odom に変換する。
    # 「[」は gz→ROS の一方向を表す。
    odometry_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        output="screen",
    )

    # /odom を購読し odom→base_link の TF を配信する。
    # 位置・姿勢の中身は gz の OdometryPublisher が生成し、上の bridge 経由で /odom に届く。
    # この TF があると RViz の Fixed Frame を odom に固定でき、機体がワープせず動いて見える。
    odom_tf_broadcaster = Node(
        package="mini_pupper_simulation",
        executable="odom_tf_broadcaster",
        output="screen",
    )

    # ロボットをスポーン。クラウチ姿勢の足が接地する高さ（オリジナルと同じ 0.10m）
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mini_pupper_2",
            "-topic", "robot_description",
            "-z", "0.10",
        ],
        output="screen",
    )

    # コントローラ起動。gz_ros2_control が YAML から自動アクティベートするため
    # spawner はフォールバック（"already active" エラーが出ても無害）
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    simple_quadruped_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_quadruped_controller"],
        output="screen",
    )

    # Stanford 歩行コントローラ（コントローラが active になってから起動）
    stanford_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("stanford_controller"),
                "stanford_controller.launch.py"])),
        launch_arguments={
            # IMU フィードバックを有効化。stanford_controller_node が /imu/data を購読し、
            # 胴体の傾き(roll/pitch)の逆向きに足先目標を回して水平へ戻す閉ループが働く。
            "orientation_from_imu": "true",
            "publish_joint_control": "true",
            "publish_states": "true",
        }.items()
    )

    # /cmd_vel → robot_command 変換（teleop_twist_keyboard 用）
    twist_converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("stanford_controller"),
                "twist_to_command_converter.launch.py"])),
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        imu_bridge,
        scan_bridge,
        odometry_bridge,
        odom_tf_broadcaster,
        description_launch,
        spawn_robot,
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=6.0, actions=[simple_quadruped_controller_spawner]),
        TimerAction(period=10.0, actions=[stanford_controller_launch]),
        TimerAction(period=14.0, actions=[twist_converter_launch]),
    ])
