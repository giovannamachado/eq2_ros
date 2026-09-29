#!/usr/bin/env python3
"""
Bridges ``/gripper_command`` (Float32) to the Kortex gripper action server.

Sits between ``fs1.gripper_control`` (which publishes 0.0/0.9) and the
robot driver's actual ``GripperCommand`` action.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32
from control_msgs.action import GripperCommand

class GripperClient(Node):
    """Forwards gripper position commands to the robot's action server."""

    def __init__(self):
        """Set up the action client and the ``/gripper_command`` subscriber."""
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
        """Send ``msg.data`` (0.0 open .. 1.0 closed) as a gripper goal."""
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
        """Log whether the robot accepted or rejected the gripper goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('A meta da garra foi REJEITADA pelo robô!')
            return
        self.get_logger().info('Meta ACEITA pelo robô, executando movimento...')

def main(args=None):
    """Entry point for the gripper action bridge node."""
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