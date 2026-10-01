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
        
        # Mantido para compatibilidade caso queira usar o antigo
        self.sub_cubo_alvo = self.create_subscription(
            String, '/ir_para_cubo', self.callback_mudar_alvo, 10
        )
        
        # Tópico responsável APENAS por atualizar a variável cubo_alvo_id
        self.sub_escolher_cubo = self.create_subscription(
            String, '/escolher_cubo', self.callback_escolher_cubo, 10
        )

        self.speed = 1.0

        # Publisher para mandar mensagens ao nó de controle de garra
        self.publisher_gripper_controller = self.create_publisher(String, '/controlador_garra', 10)

        # Avisa o fs1.supervisor o resultado do ciclo de pega ("success"/
        # "failure"), para ele saber quando sair do TELEOP (RF#06/RF#07).
        self.pick_result_pub = self.create_publisher(String, '/pick_result', 10)

        # --- Inicialização do TF2 Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Variáveis do Controlador Autônomo ---
        self.cubo_alvo_id = None
        self.fase_controle = 'ALINHAR_YZ'  # Estados: 'ALINHAR_YZ', 'AVANCAR_X', 'FECHAR_GARRA', 'RECUAR_X', 'RETORNAR_YZ'
        self.posicao_inicial = None        # Armazena (x, y, z) de onde o movimento começou
        self.contador_espera_garra = 0     # Contador para dar tempo da garra fechar
        self.executando_sequencia = False  # Flag para controlar quando o loop sequencial roda
        
        self.kp = 2.0          # Ganho proporcional
        self.v_max = 0.15      # Velocidade máxima permitida em m/s
        self.tolerancia = 0.005 # Tolerância de parada (5 milímetros)

        # Timer apontando para o controlador sequencial
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

    def loop_controle_sequencial(self):
        """
        Ciclo completo de pegada do cubo (só roda se executando_sequencia for True):
        1. ALINHAR_YZ   : Alinha Y e Z na frente do cubo target.
        2. AVANCAR_X    : Avança em X até encostar/entrar no cubo.
        3. FECHAR_GARRA : Envia comando de fechar garra e aguarda 1s.
        4. RECUAR_X     : Recua em X até o X original de partida.
        5. RETORNAR_YZ  : Retorna Y e Z até a posição original de partida.
        """
        if not self.executando_sequencia or self.cubo_alvo_id is None:
            return

        try:
            # 1. Leitura da posição atual via TF2
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'tool_frame', rclpy.time.Time()
            )
            x_atual = transform.transform.translation.x
            y_atual = transform.transform.translation.y
            z_atual = transform.transform.translation.z

            # Salva a posição inicial exata de partida se ainda não capturada
            if self.posicao_inicial is None:
                self.posicao_inicial = (x_atual, y_atual, z_atual)
                self.get_logger().info(f"📍 Posição inicial gravada: X={x_atual:.3f}, Y={y_atual:.3f}, Z={z_atual:.3f}")

            # 2. Posição Alvo do Cubo
            x_alvo, y_alvo, z_alvo = self.posicoes_cubos[self.cubo_alvo_id]
            x_init, y_init, z_init = self.posicao_inicial

            twist = Twist()

            # --- ETAPA 1: Alinhamento Y e Z ---
            if self.fase_controle == 'ALINHAR_YZ':
                erro_y = y_alvo - y_atual
                erro_z = z_alvo - z_atual
                distancia_yz = math.sqrt(erro_y**2 + erro_z**2)

                if distancia_yz <= self.tolerancia:
                    self.get_logger().info(f"🎯 Y e Z alinhados! Avançando no eixo X para o {self.cubo_alvo_id}...")
                    self.fase_controle = 'AVANCAR_X'
                    self.publisher_.publish(Twist())
                    return

                vy = self.kp * erro_y
                vz = self.kp * erro_z
                twist.linear.x = 0.0
                twist.linear.y = max(min(vy, self.v_max), -self.v_max)
                twist.linear.z = max(min(vz, self.v_max), -self.v_max)

            # --- ETAPA 2: Avanço no eixo X ---
            elif self.fase_controle == 'AVANCAR_X':
                erro_x = x_alvo - x_atual

                if abs(erro_x) <= self.tolerancia:
                    self.get_logger().info(f"✅ Chegou ao alvo! Fechando a garra...")
                    self.fase_controle = 'FECHAR_GARRA'
                    self.contador_espera_garra = 0
                    self.publisher_.publish(Twist())
                    return

                vx = self.kp * erro_x
                twist.linear.x = max(min(vx, self.v_max), -self.v_max)
                twist.linear.y = 0.0
                twist.linear.z = 0.0

            # --- ETAPA 3: Fechar a Garra ---
            elif self.fase_controle == 'FECHAR_GARRA':
                msg_garra = String()
                msg_garra.data = 'abrir'
                self.publisher_gripper_controller.publish(msg_garra)

                self.contador_espera_garra += 1
                if self.contador_espera_garra >= 20:
                    self.get_logger().info(f"✊ Garra fechada! Recuando no eixo X...")
                    self.fase_controle = 'RECUAR_X'
                return

            # --- ETAPA 4: Recuar no eixo X ---
            elif self.fase_controle == 'RECUAR_X':
                erro_x = x_init - x_atual

                if abs(erro_x) <= self.tolerancia:
                    self.get_logger().info(f"↩️ Recuo em X concluído. Retornando eixos Y e Z para a origem...")
                    self.fase_controle = 'RETORNAR_YZ'
                    self.publisher_.publish(Twist())
                    return

                vx = self.kp * erro_x
                twist.linear.x = max(min(vx, self.v_max), -self.v_max)
                twist.linear.y = 0.0
                twist.linear.z = 0.0

            # --- ETAPA 5: Retornar Y e Z para a Posição Inicial ---
            elif self.fase_controle == 'RETORNAR_YZ':
                erro_y = y_init - y_atual
                erro_z = z_init - z_atual
                distancia_yz = math.sqrt(erro_y**2 + erro_z**2)

                if distancia_yz <= self.tolerancia:
                    self.get_logger().info(f"🏠 Ciclo concluído! Robô retornou com sucesso à posição inicial.")
                    msg_garra = String()
                    msg_garra.data = 'fechar'
                    self.publisher_gripper_controller.publish(msg_garra)

                    # Reseta variáveis e desativa a sequência
                    self.executando_sequencia = False
                    self.posicao_inicial = None
                    self.fase_controle = 'ALINHAR_YZ'
                    self.publisher_.publish(Twist())
                    self.pick_result_pub.publish(String(data='success'))
                    return

                vy = self.kp * erro_y
                vz = self.kp * erro_z
                twist.linear.x = 0.0
                twist.linear.y = max(min(vy, self.v_max), -self.v_max)
                twist.linear.z = max(min(vz, self.v_max), -self.v_max)

            self.publisher_.publish(twist)

        except TransformException as ex:
            self.get_logger().warn(f"Aguardando TF2 para navegação: {ex}")

    def callback_mudar_alvo(self, msg: String):
        texto_recebido = msg.data.replace(":", "").strip()
        if texto_recebido in self.posicoes_cubos:
            self.cubo_alvo_id = texto_recebido
            self.get_logger().info(f"📌 Cubo alvo alterado para: {self.cubo_alvo_id} (Sem movimento automático)")
        else:
            self.get_logger().error(f"Alvo '{msg.data}' inválido!")

    def callback_escolher_cubo(self, msg: String):
        """Atualiza APENAS a variável cubo_alvo_id, sem executar nenhuma rotina."""
        texto_recebido = msg.data.strip()
        if texto_recebido in self.posicoes_cubos:
            self.cubo_alvo_id = texto_recebido
            self.get_logger().info(f"📌 [Tópico /escolher_cubo] cubo_alvo_id definido para: {self.cubo_alvo_id}")
        else:
            self.get_logger().warn(f"Tentativa de escolher cubo inválido: '{texto_recebido}'")

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

        # --- VERIFICAÇÃO DO COMANDO DE MÃO FECHADA ---
        if dados.get("closed") is True:
            self.get_logger().info("✊ Detectado 'closed': true no hand_status.")
            
            # Verifica se está alinhado com o cubo alvo atual
            if alinhamento and alinhamento[0] == self.cubo_alvo_id:
                self.get_logger().info(f"✅ Alinhado com o cubo alvo ({self.cubo_alvo_id}). Iniciando loop de controle sequencial!")
                self.fase_controle = 'ALINHAR_YZ'
                self.posicao_inicial = None
                self.executando_sequencia = True
                return
            else:
                self.get_logger().warn(f"❌ Não está alinhado com o cubo alvo ('{self.cubo_alvo_id}'). Executando movimento de 'frente e trás'...")
                self.executar_movimento_frente_tras_metade()
                return

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

    def executar_movimento_frente_tras_metade(self):
        """Move o robô até a metade do caminho do eixo X em relação ao cubo alvo e retorna."""
        if not self.cubo_alvo_id or self.cubo_alvo_id not in self.posicoes_cubos:
            self.get_logger().error("Nenhum cubo alvo válido definido para o movimento de frente e trás.")
            return

        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'tool_frame', rclpy.time.Time())
            x_atual = transform.transform.translation.x
            x_alvo, _, _ = self.posicoes_cubos[self.cubo_alvo_id]
            
            x_metade = x_atual + (x_alvo - x_atual) / 2.0
            
            self.get_logger().info(f"🔄 Indo até a metade do caminho em X ({x_metade:.3f})...")
            twist = Twist()
            
            # Avança até a metade
            while rclpy.ok():
                t = self.tf_buffer.lookup_transform('base_link', 'tool_frame', rclpy.time.Time())
                x_curr = t.transform.translation.x
                erro = x_metade - x_curr
                if abs(erro) < 0.005:
                    break
                twist.linear.x = max(min(self.kp * erro, self.v_max), -self.v_max)
                self.publisher_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)
            
            self.publisher_.publish(Twist())
            
            # Retorna para o ponto inicial
            self.get_logger().info("↩️ Retornando para a posição inicial de X...")
            while rclpy.ok():
                t = self.tf_buffer.lookup_transform('base_link', 'tool_frame', rclpy.time.Time())
                x_curr = t.transform.translation.x
                erro = x_atual - x_curr
                if abs(erro) < 0.005:
                    break
                twist.linear.x = max(min(self.kp * erro, self.v_max), -self.v_max)
                self.publisher_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)
                
            self.publisher_.publish(Twist())
            self.get_logger().info("✨ Movimento de frente e trás concluído.")
            self.pick_result_pub.publish(String(data='failure'))

        except TransformException as ex:
            self.get_logger().error(f"Erro no TF2 durante movimento de frente e trás: {ex}")
            self.pick_result_pub.publish(String(data='failure'))


def main(args=None):
    rclpy.init(args=args)
    node = CamTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':# pragma: no cover
    main()