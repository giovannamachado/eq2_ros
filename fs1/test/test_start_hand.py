from json import dumps
from std_msgs.msg import String
from fs1.start_hand_node import *


def test_a():
    rclpy.init(args=None)
    node = StartHandNode()
    msg = String()
    msg.data = dumps({})
    node.hand_status(msg)
    node.destroy_node()
    rclpy.shutdown()
    assert 1==1


