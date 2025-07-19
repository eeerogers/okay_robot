from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(on_exit=[Shutdown(reason="a node has exited, shutting down everything else")])
    )
    return LaunchDescription(
        [
            Node(package="okay_robot_sim", executable="mujoco_sim_node"),
            Node(package="okay_robot_control", executable="control_robot_node"),
            shutdown_on_exit,
        ]
    )
