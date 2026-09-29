"""Converts joint-angle lists (degrees) into JointTrajectory commands."""

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

import math
import time

# Serialização (JSON / String): Converta sua lista em uma string usando json.dumps() 
# em Python antes de publicar no tópico std_msgs/String,
#  e faça o inverso (json.loads()) ao receber.
    
def conversor_graus_radianos(lista):
    """Convert a list of angles in degrees to radians, same order."""
    resultado = []
    for valor in lista:
        resultado.append(math.radians(valor))
    return resultado

class SensorNode(Node):
    """Bridges ``/posicoes_garra`` (JSON degrees) to the joint trajectory controller."""

    def __init__(self):
        """Set up the trajectory publisher and the ``/posicoes_garra`` subscriber."""
        super().__init__('transmissor_de_posicoes')
        self.get_logger().info('transmissor de posicoes esta ligado')      
        
        # Publisher para joint trajectory, criando e dizendo sua linha de publicaçaao
        self.publisher_joint_trajectory = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        #publicador para ativar a garra
        self.publisher_gripper_controller = self.create_publisher(String,'/controlador_garra',10)
        
        #escutador da kinova_api
        self.subscriber_kinova_api = self.create_subscription(String,'/posicoes_garra', self.comando_garra ,10)


        #comando para utilizar o fluxo antigo
        #self.enviar_posicao()
                            #equivalente a mensagem
    def comando_garra(self, recebida):
        """Publish ``recebida`` (JSON list of 6 angles, degrees) as a 5 s trajectory."""
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        lista_graus = json.loads(recebida.data)
        print(f'recebi a mensagem{lista_graus}')
        point = JointTrajectoryPoint() 
        point.positions = conversor_graus_radianos(lista_graus) #colocar coordenada aqui
        point.time_from_start.sec=5
        msg.points=[point]
        self.publisher_joint_trajectory.publish(msg)

    
def main(args=None):
   """Entry point for the joint-trajectory bridge node."""
   rclpy.init(args=args)
   sensor_node = SensorNode()
   rclpy.spin(sensor_node)
   sensor_node.destroy_node()
   rclpy.shutdown()
