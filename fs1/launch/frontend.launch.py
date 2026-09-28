"""
Launch the ROS side of the Flutter front-end integration.

Starts rosbridge (WebSocket for Flutter), the supervisor FSM, the existing
vision / joint / gripper nodes and republishes the camera as JPEG so the
front-end can render it over the same WebSocket.

Arguments:
    rosbridge_port: WebSocket port used by the Flutter app (default 9090).
    home_settle_s / scan_window_s: FSM timings, see ``fs1/supervisor.py``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the front-end integration."""
    pkg = 'fs1'

    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('home_settle_s', default_value='7.0'),
        DeclareLaunchArgument('scan_window_s', default_value='2.0'),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': LaunchConfiguration('rosbridge_port')}],
        ),
        Node(
            package=pkg, executable='supervisor', name='supervisor',
            output='screen',
            parameters=[{
                'home_settle_s': LaunchConfiguration('home_settle_s'),
                'scan_window_s': LaunchConfiguration('scan_window_s'),
            }],
        ),
        Node(package=pkg, executable='my_node', name='vision_node',
             output='screen'),
        Node(package=pkg, executable='joints_control', name='joints_control',
             output='screen'),
        Node(package=pkg, executable='gripper_control', name='gripper_control',
             output='screen'),
        Node(package=pkg, executable='gripper_client', name='gripper_client'),

        # /camera/image (raw, from my_node) -> JPEG for the Flutter app.
        Node(
            package='image_transport',
            executable='republish',
            name='effector_camera_jpeg',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/camera/image'),
                ('out/compressed', '/camera/effector/image_raw/compressed'),
            ],
        ),

        # /camera/operator/image_raw (raw, from hand_node) -> JPEG for the
        # Flutter app. hand_node itself is NOT launched here: it needs
        # mediapipe, which this image does not ship (see eq2_ros/reademe.md
        # for how to run it manually with the dev venv). This republisher
        # is harmless to leave running even when hand_node is off: it just
        # sits idle until something publishes to its input topic.
        Node(
            package='image_transport',
            executable='republish',
            name='operator_camera_jpeg',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/camera/operator/image_raw'),
                ('out/compressed', '/camera/operator/image_raw/compressed'),
            ],
        ),
    ])
