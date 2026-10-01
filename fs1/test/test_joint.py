from json import dumps
from std_msgs.msg import String
import pytest
from fs1.joints_control import *


def test_a():
    rclpy.init(args=None)
    node = SensorNode()
    msg = String()
    msg.data = dumps([10,20,30])
    rclpy.spin_once(node,timeout_sec=1.5)
    node.comando_garra(msg)
    node.destroy_node()
    rclpy.shutdown()
    assert 1==1

def test_rad():
    original = [90,120,45,33,55]
    result = conversor_graus_radianos(original)
    assert all(result[i]==math.radians(original[i]) for i in range(len(original)))

