import cv2

from fs1.hand_node import *
import pytest
import rclpy
from random import randint,random
from std_msgs.msg import String

@pytest.fixture
def hDetect():
    instancia = HandNode(cross_mode=True,frame_jump=3)
    return instancia
@pytest.fixture
def handD():
    hand = {i:[randint(1,100)*random() for _ in range(4)] for i in range(5)}
    instancia = handDist(0.5,1.0,finger_dists=hand,comp_fingers={2:1.0})
    return instancia
@pytest.mark.parametrize(
   "a, esperado",
   [(0.5, True),(1, False),],
   ids=["a", "b"],)
def test_handDist(handD,a,esperado):
    print(handD)
    assert (handD.x == a) == esperado

def test_Detector(args=None):
    rclpy.init(args=args)
    limit = 1
    node = HandNode(cross_mode=True,frame_jump=3,test_mode={"limit":limit})
    
    try:rclpy.spin_once(node,timeout_sec=limit+1)
        
    except KeyboardInterrupt: pass
    finally:
        msg = String()
        msg.data = json.dumps({"limit":limit+30,"rez":360,"mzone":60,"dzone":60,"frame_jump":6})
        node.switch_running(msg)
        dist = node.hand_dist
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
    assert dist.closed != None
