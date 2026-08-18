"""

  ros2 launch can_odom_ros demo.launch.py
  ros2 launch can_odom_ros demo.launch.py scenario:=square
  # 실물 연결 시 (PCAN-USB → can0)
  ros2 launch can_odom_ros demo.launch.py with_simulator:=false can_backend:=socketcan run_scenario:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("can_odom_ros")

    args = [
        DeclareLaunchArgument("can_backend", default_value="virtual",
                              description="virtual | socketcan | vcan | pcan"),
        DeclareLaunchArgument("with_simulator", default_value="true",
                              description="가상 2ELD2 드라이브 동시 기동"),
        DeclareLaunchArgument("run_scenario", default_value="true",
                              description="대본 cmd_vel 퍼블리셔 실행"),
        DeclareLaunchArgument("scenario", default_value="default",
                              choices=["default", "square"]),
        DeclareLaunchArgument("hardware_config", default_value=os.path.join(
            share, "config", "hardware.yaml")),
        DeclareLaunchArgument("profile_config", default_value=os.path.join(
            share, "config", "profiles", "cia402_leadshine_2eld2.yaml")),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
    ]

    driver = Node(
        package="can_odom_ros", executable="driver_node", name="can_odom_driver",
        output="screen",
        parameters=[{
            "can_backend": LaunchConfiguration("can_backend"),
            "with_simulator": LaunchConfiguration("with_simulator"),
            "hardware_config": LaunchConfiguration("hardware_config"),
            "profile_config": LaunchConfiguration("profile_config"),
            "publish_tf": LaunchConfiguration("publish_tf"),
        }],
    )

    scenario = Node(
        package="can_odom_ros", executable="scenario_node", name="scenario_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_scenario")),
        parameters=[{"scenario": LaunchConfiguration("scenario"), "loop": False}],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(share, "rviz", "odom.rviz")],
    )

    return LaunchDescription(args + [driver, scenario, rviz])
