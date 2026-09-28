"""
A ROS 2 node for keyboard teleoperation.

Captures keyboard inputs in raw mode from the terminal and publishes 
standard geometry_msgs/Twist messages to the '/cmd_vel' topic.
"""

import sys
import select
import termios
import tty
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CamTeleop(Node):
    """
    A standalone ROS 2 node that reads keystrokes and publishes Twist messages.
    
    Attributes:
        publisher_ (Publisher): Publishes twist commands to '/cmd_vel'.
        speed (float): Current translation speed in meters per second.
        target_y (float): Target velocity along the Y-axis.
        target_z (float): Target velocity along the Z-axis.
    """

    def __init__(self):
        """Initializes the teleop node and its publisher."""
        super().__init__('keyboard_teleop')
        self.subscriber_cam_node = self.create_subscription(String, '/hand_status', self.recebi_mensagem,10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 1.0
        self.target_y = 0.0
        self.target_z = 0.0

    def recebi_mensagem(self, mensagem: String):
        """
        Callback que recebe uma string JSON com os comandos de movimento e garras,
        converte os dados e publica a velocidade correspondente via Twist.
        """
        try:
            # Decodifica a mensagem JSON em um dicionário Python
            dados = json.loads(mensagem.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Falha ao decodificar o JSON recebido: {mensagem.data}")
            return

        twist = Twist()

        
        if 'esquerda' in dados:
            # 'esquerda' em dados representa magnitude positiva no sentido +Y
            twist.linear.y = float(dados['esquerda']) * 0.1 * self.speed
        elif 'direita' in dados:
            # 'direita' em dados representa magnitude positiva no sentido -Y
            twist.linear.y = -float(dados['direita']) * 0.1 * self.speed
        else:
            twist.linear.y = 0.0

        # --- Controle do Eixo Z (Cima / Baixo) ---
        if 'cima' in dados:
            # 'cima' em dados representa magnitude positiva no sentido +Z
            twist.linear.z = float(dados['cima']) * 0.1 * self.speed
        elif 'baixo' in dados:
            # 'baixo' em dados representa magnitude positiva no sentido -Z
            twist.linear.z = -float(dados['baixo']) * 0.1 * self.speed
        else:
            twist.linear.z = 0.0

        # Publica o comando de velocidade
        self.publisher_.publish(twist)

def main(args=None):
    """Entry point for the Keyboard Teleop node."""
    rclpy.init(args=args)
    node = CamTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()