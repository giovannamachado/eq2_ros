import json
import math
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
        
        # --- Posições Alvo dos Cubos na Prateleira (X, Y, Z) ---
        self.posicoes_cubos = {
            "Cubo 01": (0.618,  0.232, 0.309),
            "Cubo 02": (0.618,  0.133, 0.309),
            "Cubo 03": (0.618,  0.017, 0.309),
            "Cubo 04": (0.618, -0.087, 0.309),
            "Cubo 05": (0.618,  0.230, 0.157),
            "Cubo 06": (0.618,  0.128, 0.157),
            "Cubo 07": (0.618,  0.015, 0.157),
            "Cubo 08": (0.618, -0.098, 0.157),
        }
        # Margens de Tolerância para considerar "Alinhado" (em metros)
        self.tolerancia_y = 0.2 # ± 2 cm
        self.tolerancia_z = 0.2  # ± 2 cm
        self.raio_maximo_2d = 0.025  # Raio máximo no plano YZ (2.5 cm)

    def verificar_alinhamento_cubo(self) -> tuple | None:
        """
        Consulta a posição atual do 'tool_frame' via TF2 e verifica
        se ele está dentro da área de tolerância de algum cubo cadastrado.

        :return: Tupla com (nome_do_cubo, distancia_metros) se alinhado, senão None.
        """
        try:
            # Obtém a transformação atual do base_link para o tool_frame
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool_frame',
                rclpy.time.Time()
            )
            x_atual = transform.transform.translation.x
            y_atual = transform.transform.translation.y
            z_atual = transform.transform.translation.z

            # Testa a posição atual contra cada um dos cubos
            for nome_cubo, (_, y_alvo, z_alvo) in self.posicoes_cubos.items():
                # 1. Checagem por caixa (eixos individuais)
                em_alcance_y = abs(y_atual - y_alvo) <= self.tolerancia_y
                em_alcance_z = abs(z_atual - z_alvo) <= self.tolerancia_z

                #2. Distância Euclidiana 2D no plano YZ
                distancia_2d = math.sqrt((y_atual - y_alvo)**2 + (z_atual - z_alvo)**2)
                if em_alcance_y and em_alcance_z and distancia_2d <= self.raio_maximo_2d:
                    return nome_cubo, distancia_2d, (y_atual, z_atual)

        except TransformException as ex:
            self.get_logger().debug(f"Não foi possível obter TF2 para checagem: {ex}")

        return None


    def recebi_mensagem(self, mensagem: String):
        try:
            dados = json.loads(mensagem.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Falha ao decodificar o JSON recebido: {mensagem.data}")
            return

        alinhamento = self.verificar_alinhamento_cubo()
        if alinhamento:
            cubo, dist, pos = alinhamento
            self.get_logger().info(
                f" ALINHADO COM {cubo}! Erro:"
            )
        
    
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