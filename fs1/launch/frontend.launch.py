"""
Launch the ROS side of the Flutter front-end integration.

Starts rosbridge (WebSocket for Flutter), the supervisor FSM, the vision /
joint control nodes, and republishes the effector camera as JPEG so the
front-end can render it over the same WebSocket.

Run alongside ``ros2 launch fs1 claw_machine.launch.py`` for the physical
robot side (MoveIt Servo, servo_adapter, cam_teleop, gripper_client,
gripper_control, hand_node). The two together are the full system; this
file deliberately does NOT also start servo_adapter/cam_teleop/
gripper_client/gripper_control itself anymore — running both of each at
once means two independent processes both reacting to the same
/hand_status and both publishing /cmd_vel, which is a real safety risk on
the physical robot (two controllers fighting over the same motion), not
just a duplicate-node warning.

NOT started here:
* MoveIt Servo, servo_adapter, cam_teleop, gripper_client, gripper_control
  — all in ``claw_machine.launch.py`` now; see above.
* ``hand_node`` — needs mediapipe, which this image does not ship (run it
  manually with the dev venv; see eq2_ros/reademe.md).

Arguments:
    rosbridge_port: WebSocket port used by the Flutter app (default 9090).
        Change it if another process on the same machine already holds 9090
        (the container uses --net host, so the port is shared with the
        whole machine, not just this container).
    camera_index: /dev/video<N> (or a stable /dev/v4l/by-id/... path, which
        survives the camera moving to a different USB port) used by the
        effector (shelf/ArUco) camera. Default 0. On a machine with more
        than one camera, point this at the robot's 2K camera; see
        fs1/fs1/hand_node.py's own camera_index parameter for the operator
        webcam.
    home_settle_s / scan_window_s: FSM timings, see ``fs1/supervisor.py``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the launch description for the front-end integration."""
    pkg = 'fs1'

    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('camera_index', default_value='0'),
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
        Node(package=pkg, executable='vision_node', name='vision_node',
             output='screen',
             # ParameterValue(..., value_type=str) força a string: sem isso,
             # um valor puramente numérico (ex. o "0" padrão) vira um YAML
             # inteiro no arquivo de parâmetros do launch e o nó morre, já
             # que vision_node.py declara camera_index como string (para
             # também aceitar um caminho /dev/v4l/by-id/...).
             parameters=[{'camera_index': ParameterValue(
                 LaunchConfiguration('camera_index'), value_type=str)}]),
        # Usado pelo supervisor só para as transições de HOME (RF#01). O
        # gripper_control/gripper_client e o servo_adapter/cam_teleop da
        # pega em si agora vêm só do claw_machine.launch.py (ver docstring).
        Node(package=pkg, executable='joints_control', name='joints_control',
             output='screen'),

        # /camera/image (raw, from vision_node) -> JPEG for the Flutter app.
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
