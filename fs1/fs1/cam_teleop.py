import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Importações necessárias do TF2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class CamTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.subscriber_cam_node = self.create_subscription(String, '/hand_status', self.recebi_mensagem, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.speed = 1.0

        # --- Inicialização do TF2 Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Definição dos Limites (em metros no referencial do base_link) ---
        self.y_min = -0.171  # Limite máximo para a DIREITA (-Y)
        self.y_max =  0.297  # Limite máximo para a ESQUERDA (+Y)
        self.z_min =  0.156  # Limite mínimo para BAIXO (+Z)
        self.z_max =  0.357  # Limite máximo para CIMA (+Z)

    def recebi_mensagem(self, mensagem: String):
        try:
            dados = json.loads(mensagem.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Falha ao decodificar o JSON recebido: {mensagem.data}")
            return

        twist = Twist()

        # 1. Cálculo inicial da velocidade desejada
        if 'esquerda' in dados:
            twist.linear.y = float(dados['esquerda']) * 0.1 * self.speed
        elif 'direita' in dados:
            twist.linear.y = -float(dados['direita']) * 0.1 * self.speed

        if 'cima' in dados:
            twist.linear.z = float(dados['cima']) * 0.1 * self.speed
        elif 'baixo' in dados:
            twist.linear.z = -float(dados['baixo']) * 0.1 * self.speed

        # 2. Obtenção da posição atual via TF2
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'tool_frame',
                rclpy.time.Time()
            )
            
            pos_y = t.transform.translation.y
            pos_z = t.transform.translation.z

            
            # Trava do Eixo Y (Esquerda / Direita)
            if pos_y >= self.y_max and twist.linear.y > 0.0:
                self.get_logger().warn("Limite máximo em Y (Esquerda) atingido!")
                twist.linear.y = 0.0
            elif pos_y <= self.y_min and twist.linear.y < 0.0:
                self.get_logger().warn("Limite mínimo em Y (Direita) atingido!")
                twist.linear.y = 0.0

            # Trava do Eixo Z (Cima / Baixo)
            if pos_z >= self.z_max and twist.linear.z > 0.0:
                self.get_logger().warn("Limite máximo em Z (Cima) atingido!")
                twist.linear.z = 0.0
            elif pos_z <= self.z_min and twist.linear.z < 0.0:
                self.get_logger().warn("Limite mínimo em Z (Baixo) atingido!")
                twist.linear.z = 0.0

        except TransformException as ex:
            # Se o TF falhar temporariamente (ex: inicialização), interrompe movimento por segurança
            self.get_logger().error(f"Não foi possível obter a transformação base_link -> tool_frame: {ex}")
            twist.linear.y = 0.0
            twist.linear.z = 0.0

        # 4. Publica a velocidade filtrada
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