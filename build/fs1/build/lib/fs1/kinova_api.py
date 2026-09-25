import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

pose_foto = [6.0,53.0,146.0,-33.0,7.0,33]
home= [-19.0, 2.0, 131.0, 61.0, -44.0, -68.0] #agora tenho liberdade total
pre_place = [-98.0,-33.0,83.0,70.0,-29.0,-70.0]
place=[-98.0,-48.0,89.0,78.0,-49.0,-80.0]

cubos=[]
cubos.append([18.0,-43.0,78.0,122.0,-36.0,-117.0])#cubo 1
cubos.append([-1.0,-36.0,85.0,88.0,-32.0,-88.0])#cubo 2
cubos.append([-14.0,-37.0,76.0,57.0,-28.0,-60.0])#cubo 3
cubos.append([-25.0,-46.0,55.0,24.0,-28.0,-26.0])#cubo 4
cubos.append([19.0,-59.0,86.0,113.0,-57.0,-104.0])#cubo 5
cubos.append([18.0,-63.0,77.0,113.0,-52.0,-106.0])#cubo 6
cubos.append([-14.0,-55.0,85.0,72.0,-52.0,-79.0])#cubo 7
cubos.append([-29.0,-59.0,69.0,48.0,-47.0,-59.0])#cubo 8

#adicionar todos os locais de cubos
class KinovaApi(Node):
    def __init__(self):
        super().__init__('api_kinova')
        self.get_logger().info('orquestrador de nós iniciado')      
        
        # Publisher para mandar mensagems ao nó controle de juntas
        self.publisher_controlador_juntas = self.create_publisher(String, '/posicoes_garra', 10)
        
        #Publisher para mandar mensagens ao nó de controle de garra
        self.publisher_gripper_controller = self.create_publisher(String,'/controlador_garra',10)

    def start(self):
        time.sleep(2)
        msg_junta = String()
        msg_junta.data = json.dumps(pose_foto)
        self.publisher_controlador_juntas.publish(msg_junta)
        time.sleep(10)#esperando tirar a foto
        msg_junta.data = json.dumps(home)
        self.publisher_controlador_juntas.publish(msg_junta)#ficando na posição de home
        time.sleep(7)
        msg_garra = String()
        msg_garra.data = 'abrir'
        self.publisher_gripper_controller.publish(msg_garra)
        time.sleep(6)

    def pick_one_cube(self):
        time.sleep(2)
        cubo = 1
        msg_junta = String()
        msg_junta.data=json.dumps(cubos[cubo])
        self.publisher_controlador_juntas.publish(msg_junta)
        time.sleep(9)
        msg = String()
        msg.data='fechar'
        self.publisher_gripper_controller.publish(msg)
        time.sleep(3)
        msg_junta_home = String()
        msg_junta_home.data = json.dumps(home)
        self.publisher_controlador_juntas.publish(msg_junta_home)
        time.sleep(7)

    def test_abrir_fechar(self):
        time.sleep(4)
        msg = String()
        msg.data = 'fechar'
        self.publisher_gripper_controller.publish(msg)
        time.sleep(5)
        msg = String()
        msg.data = 'abrir' 

    def put_in_box_function(self):
        msg = String()
        msg.data = json.dumps(pre_place)
        self.publisher_controlador_juntas.publish(msg)
        time.sleep(9)
        msg = String()
        msg.data = json.dumps(place)
        self.publisher_controlador_juntas.publish(msg)
        time.sleep(9)
        msg.data='abrir'
        self.publisher_gripper_controller.publish(msg)
        msg=String()
        msg.data = json.dumps(home)
        self.publisher_controlador_juntas.publish(msg)
        
        
def main(args=None):
   rclpy.init(args=args)
   kinova_instance = KinovaApi()
   kinova_instance.start()
   kinova_instance.pick_one_cube()
   kinova_instance.put_in_box_function()
   rclpy.spin(kinova_instance)
   kinova_instance.destroy_node()
   rclpy.shutdown()