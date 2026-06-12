from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    settle_velocity_threshold = LaunchConfiguration('settle_velocity_threshold')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'settle_velocity_threshold',
            default_value='0.033',
            description='Joint velocity threshold used to decide when the arm is settled'
        ),

        Node(
            package='piperx_perception',
            executable='aruco_sim_detector',
            name='aruco_sim_detector',
            output='screen'
        ),

        Node(
            package='piperx_control',
            executable='piperx_sim_control',
            name='piperx_sim_control',
            output='screen',
            parameters=[
                {'settle_velocity_threshold': settle_velocity_threshold}
            ]
        )
    ])
