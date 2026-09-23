import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time

class gripperControl(Node):
    def __init__(self):
       
        super().__init__('transmissor_da_garra')
        self.get_logger().info('transmissor da garra esta ligado')     
        self.get_logger().info("entrei aqui") 

        # Publisher para joint trajectory, criando e dizendo sua linha de publicaçaao
        self.publisher_gripper = self.create_publisher(Float32,'/gripper_command',10)
        self.subscriber_gripper_client = self.create_subscription(String,'/controlador_garra', self.recebeu_mensagem ,10)
    

    def recebeu_mensagem(self, mensagem: String):
        if(mensagem.data == 'fechar'):
            self.fechar_garra()
        if(mensagem.data == 'abrir'):          
            self.abrir_garra()
    
    def fechar_garra(self):
        msg=Float32()
        msg.data = 0.0
        self.publisher_gripper.publish(msg)

    def abrir_garra(self):
        msg=Float32()
        msg.data = 0.9
        self.publisher_gripper.publish(msg)
    
def main(args=None):
   rclpy.init(args=args)
   gripper_control = gripperControl()
   rclpy.spin(gripper_control)
   gripper_control.destroy_node()
   rclpy.shutdown()
