import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

# Importações necessárias do TF2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class CamTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.subscriber_cam_node = self.create_subscription(String, '/hand_status', self.recebi_mensagem, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Tópico para receber o comando de mudar de cubo
        self.sub_cubo_alvo = self.create_subscription(
            String, '/ir_para_cubo', self.callback_mudar_alvo, 10
        )
        self.speed = 1.0

        # --- Inicialização do TF2 Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Variáveis do Controlador Autónomo ---
        self.cubo_alvo_id = None
        self.fase_controle = 'ALINHAR_YZ'  # Modos: 'ALINHAR_YZ' ou 'AVANCAR_X'
        self.kp = 2.0          # Ganho proporcional
        self.v_max = 0.15      # Velocidade máxima permitida em m/s
        self.tolerancia = 0.005 # Tolerância de parada (5 milímetros)

        # Timer apontando para o NOVO controlador sequencial
        self.control_timer = self.create_timer(0.05, self.loop_controle_sequencial)

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
        self.tolerancia_y = 0.2
        self.tolerancia_z = 0.2
        self.raio_maximo_2d = 0.025




    def loop_controle_autonomo(self):
        """Malha de controle proporcional para mover o robô até o cubo selecionado."""
        if self.cubo_alvo_id is None:
            return  # Nenhum alvo ativo

        try:
            # 1. Leitura da posição atual
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'tool_frame', rclpy.time.Time()
            )
            x_atual = transform.transform.translation.x
            y_atual = transform.transform.translation.y
            z_atual = transform.transform.translation.z

            # 2. Posição Alvo
            x_alvo, y_alvo, z_alvo = self.posicoes_cubos[self.cubo_alvo_id]

            # 3. Cálculo dos erros por eixo
            erro_y = y_alvo - y_atual
            erro_z = z_alvo - z_atual
            erro_x = x_alvo - x_atual

            # Erro de distância nos eixos Y e Z
            distancia_yz = math.sqrt(erro_y**2 + erro_z**2)

            # 4. Condição de Parada (Chegou ao Alvo)
            if distancia_yz <= self.tolerancia:
                self.get_logger().info(f"Alvo atingido! Cubo {self.cubo_alvo_id} alcançado.")
                self.cubo_alvo_id = None
                self.publisher_.publish(Twist())  # Publica 0 para parar o robô
                return

            # 5. Controlador Proporcional (P) com Saturação de Velocidade
            twist = Twist()
            
            # Velocidades calculadas
            vy = self.kp * erro_y
            vz = self.kp * erro_z
            vx = self.kp * erro_x

            # Aplicação dos limites de velocidade (+/- v_max)
            twist.linear.y = max(min(vy, self.v_max), -self.v_max)
            twist.linear.z = max(min(vz, self.v_max), -self.v_max)
            twist.linear.x = max(min(vx, self.v_max), -self.v_max)

            # Publica os comandos de velocidade para o robô
            self.publisher_.publish(twist)

        except TransformException as ex:
            self.get_logger().warn(f"Aguardando TF2 para navegação: {ex}")
    

    def loop_controle_sequencial(self):
        """
        Controlador em duas etapas:
        1ª Etapa ('ALINHAR_YZ'): Ajusta apenas a altura (Z) e posição lateral (Y), com Vx = 0.
        2ª Etapa ('AVANCAR_X'): Com Y e Z já alinhados, avança exclusivamente no eixo X.
        """
        if self.cubo_alvo_id is None:
            return  # Nenhum alvo ativo

        try:
            # 1. Leitura da posição atual via TF2
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'tool_frame', rclpy.time.Time()
            )
            x_atual = transform.transform.translation.x
            y_atual = transform.transform.translation.y
            z_atual = transform.transform.translation.z

            # 2. Posição Alvo
            x_alvo, y_alvo, z_alvo = self.posicoes_cubos[self.cubo_alvo_id]

            # 3. Cálculo dos erros por eixo
            erro_x = x_alvo - x_atual
            erro_y = y_alvo - y_atual
            erro_z = z_alvo - z_atual

            distancia_yz = math.sqrt(erro_y**2 + erro_z**2)
            twist = Twist()

            # --- ETAPA 1: Alinhamento em Y e Z ---
            if self.fase_controle == 'ALINHAR_YZ':
                if distancia_yz <= self.tolerancia:
                    self.get_logger().info(f"🎯 Eixos Y e Z alinhados! Iniciando avanço no eixo X para o {self.cubo_alvo_id}...")
                    self.fase_controle = 'AVANCAR_X'
                    self.publisher_.publish(Twist())  # Breve parada de transição
                    return

                # Calcula apenas Vy e Vz (Vx zerado)
                vy = self.kp * erro_y
                vz = self.kp * erro_z

                twist.linear.x = 0.0
                twist.linear.y = max(min(vy, self.v_max), -self.v_max)
                twist.linear.z = max(min(vz, self.v_max), -self.v_max)

            # --- ETAPA 2: Avanço no eixo X ---
            elif self.fase_controle == 'AVANCAR_X':
                if abs(erro_x) <= self.tolerancia:
                    self.get_logger().info(f"✅ Alvo atingido! {self.cubo_alvo_id} alcançado com sucesso.")
                    self.cubo_alvo_id = None
                    self.fase_controle = 'ALINHAR_YZ'  # Reset de estado
                    self.publisher_.publish(Twist())   # Parar o robô
                    return

                # Calcula apenas Vx (Vy e Vz zerados)
                vx = self.kp * erro_x

                twist.linear.x = max(min(vx, self.v_max), -self.v_max)
                twist.linear.y = 0.0
                twist.linear.z = 0.0

            # Publica o comando de velocidade correspondente à etapa atual
            self.publisher_.publish(twist)

        except TransformException as ex:
            self.get_logger().warn(f"Aguardando TF2 para navegação: {ex}")

    def callback_mudar_alvo(self, msg: String):
        """Callback acionado ao receber a mensagem de seleção no tópico /ir_para_cubo."""
        # Trata entrada em texto (ex: "Cubo: 04" ou "Cubo 04")
        texto_recebido = msg.data.replace(":", "").strip()
        
        if texto_recebido in self.posicoes_cubos:
            self.cubo_alvo_id = texto_recebido
            self.fase_controle = 'ALINHAR_YZ'  # Garante que a sequência sempre inicie pelo YZ
            self.get_logger().info(f"🤖 Novo alvo definido: {self.cubo_alvo_id}. Iniciando alinhamento Y/Z...")
        else:
            self.get_logger().error(f"Alvo '{msg.data}' inválido! Use chaves como 'Cubo 01' até 'Cubo 08'.")

    def verificar_alinhamento_cubo(self) -> tuple | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool_frame',
                rclpy.time.Time()
            )
            x_atual = transform.transform.translation.x
            y_atual = transform.transform.translation.y
            z_atual = transform.transform.translation.z

            for nome_cubo, (_, y_alvo, z_alvo) in self.posicoes_cubos.items():
                em_alcance_y = abs(y_atual - y_alvo) <= self.tolerancia_y
                em_alcance_z = abs(z_atual - z_alvo) <= self.tolerancia_z
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
            self.get_logger().info(f"ALINHADO COM {cubo}!")

        twist = Twist()

        if 'esquerda' in dados:
            twist.linear.y = float(dados['esquerda']) * 0.1 * self.speed
        elif 'direita' in dados:
            twist.linear.y = -float(dados['direita']) * 0.1 * self.speed

        if 'cima' in dados:
            twist.linear.z = float(dados['cima']) * 0.1 * self.speed
        elif 'baixo' in dados:
            twist.linear.z = -float(dados['baixo']) * 0.1 * self.speed

        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'tool_frame',
                rclpy.time.Time()
            )
            pos_y = t.transform.translation.y
            pos_z = t.transform.translation.z

            if pos_y >= self.y_max and twist.linear.y > 0.0:
                self.get_logger().warn("Limite máximo em Y (Esquerda) atingido!")
                twist.linear.y = 0.0
            elif pos_y <= self.y_min and twist.linear.y < 0.0:
                self.get_logger().warn("Limite mínimo em Y (Direita) atingido!")
                twist.linear.y = 0.0

            if pos_z >= self.z_max and twist.linear.z > 0.0:
                self.get_logger().warn("Limite máximo em Z (Cima) atingido!")
                twist.linear.z = 0.0
            elif pos_z <= self.z_min and twist.linear.z < 0.0:
                self.get_logger().warn("Limite mínimo em Z (Baixo) atingido!")
                twist.linear.z = 0.0

        except TransformException as ex:
            self.get_logger().error(f"Não foi possível obter a transformação base_link -> tool_frame: {ex}")
            twist.linear.y = 0.0
            twist.linear.z = 0.0

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CamTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':# pragma: no cover
    main()