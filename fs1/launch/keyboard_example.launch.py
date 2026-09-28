"""
Launch file to start the Keyboard Teleop and the Servo Adapter nodes together.

This launch file opens a separate terminal window (using xterm) for the keyboard 
node to ensure that sys.stdin captures keystrokes correctly without interfering 
with the ROS 2 launch log manager.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generates the launch description containing both nodes."""
    
    pkg_name = 'fs1'

    # Node que traduz os comandos do teclado para mensagens de controle do robô
    servo_adapter_node = Node(
        package=pkg_name,
        executable='servo_adapter',
        name='servo_adapter',
        output='screen'
    )

    # Node que lê os comandos do teclado e publica mensagens de controle
    keyboard_teleop_node = Node(
        package=pkg_name,
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -title "KORTEX KEYBOARD TELEOP" -geometry 60x20 -hold -e'
    )

    return LaunchDescription([
        servo_adapter_node,
        keyboard_teleop_node
    ])