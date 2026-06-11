from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    aruco_detector = Node(
        package='piperx_perception',
        executable='aruco_sim_detector',
        name='aruco_sim_detector',
        output='screen'
    )

    piperx_control = Node(
        package='piperx_control',
        executable='piperx_sim_control',
        name='piperx_sim_control',
        output='screen'
    )

    return LaunchDescription([
        aruco_detector,
        piperx_control
    ])
