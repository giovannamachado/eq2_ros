#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32
from control_msgs.action import GripperCommand

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_bridge_node')
        
        # Altere o tópico da action caso 'ros2 action list' mostre um nome diferente
        self.action_topic = '/gen3_lite_2f_gripper_controller/gripper_cmd'
        
        self._action_client = ActionClient(
            self, 
            GripperCommand, 
            self.action_topic
        )
        
        self._subscription = self.create_subscription(
            Float32,
            '/gripper_command',
            self.topic_callback,
            10
        )
        
        self.get_logger().info(f'Gripper bridge iniciado. Aguardando servidor: {self.action_topic}')

    def topic_callback(self, msg):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'Servidor de Action {self.action_topic} NÃO está online!')
            return
            
        goal_msg = GripperCommand.Goal()
        # Kinova Gen3 Lite: 0.0 (aberto) até 1.0 (fechado)
        goal_msg.command.position = float(msg.data)
        goal_msg.command.max_effort = 80.0  # Esforço máximo
        
        self.get_logger().info(f'Enviando meta para a garra: Posição = {goal_msg.command.position}')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('A meta da garra foi REJEITADA pelo robô!')
            return
        self.get_logger().info('Meta ACEITA pelo robô, executando movimento...')

def main(args=None):
    rclpy.init(args=args)
    bridge_node = GripperClient()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()