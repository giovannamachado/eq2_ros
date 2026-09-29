import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_name = 'fs1'

    # --- 1. Configurações do MoveIt Servo ---
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="gen3_lite",
            package_name="kinova_gen3_lite_moveit_config"
        )
        .robot_description(file_path=os.path.join(
            get_package_share_directory("kortex_description"), 
            "robots", 
            "gen3_lite_gen3_lite_2f.xacro"
        ))
        .robot_description_semantic(file_path="config/gen3_lite.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    servo_yaml_path = os.path.join(
        get_package_share_directory('kortex_servo'),
        'config',
        'servo_config.yaml'
    )
    with open(servo_yaml_path, 'r') as file:
        servo_yaml = yaml.safe_load(file)
    
    servo_params = {"moveit_servo": servo_yaml}

    # --- 2. Definição dos Nós ---

    # Nó do MoveIt Servo (C++)
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': True}          
        ],
        output='screen'
    )

    # Nó Servo Adapter
    servo_adapter_node = Node(
        package=pkg_name,
        executable='servo_adapter',
        name='servo_adapter',
        output='screen'
    )

    # Nó Teleop da Câmera
    cam_teleop_node = Node(
        package=pkg_name,
        executable='CamTeleop',
        name='CamTeleop',
        output='screen'
    )

    # Nó da Mão (Processamento)
    hand_node = Node(
        package=pkg_name,
        executable='hand_node',
        name='hand_node',
        output='screen'
    )

    # Nó de Inicialização da Mão
    start_hand_node = Node(
        package=pkg_name,
        executable='start_hand_node',
        name='start_hand_node',
        output='screen'
    )

    # --- 3. Retorno Unificado ---
    return LaunchDescription([
        servo_node,
        servo_adapter_node,
        cam_teleop_node,
        hand_node,
        start_hand_node
    ])