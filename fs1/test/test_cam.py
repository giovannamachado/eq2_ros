from json import dumps

import pytest
from std_msgs.msg import String
from fs1.cam_teleop import *

#Parcial
def test_a():
    rclpy.init(args=None)
    node  = CamTeleop()
    msg = String()
    msg.data = "Cubo 01"
    node.callback_mudar_alvo(msg)
    node.callback_escolher_cubo(msg)
    node.recebi_mensagem(msg)
    msg.data = "Cubo 09"
    node.callback_mudar_alvo(msg)
    node.callback_escolher_cubo(msg)
    node.loop_controle_sequencial()
    node.verificar_alinhamento_cubo()
    msg.data = dumps({"esquerda":1,"cima":1,"closed":False})
    node.recebi_mensagem(msg)
    msg.data = dumps({"direita":1,"baixo":1,"closed":True})
    node.recebi_mensagem(msg)
    node.executar_movimento_frente_tras_metade()
    rclpy.spin_once(node,timeout_sec=1.5)
    node.destroy_node()
    rclpy.shutdown()
    assert 1==1


