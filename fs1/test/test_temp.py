from json import dumps

import pytest
from std_msgs.msg import String
from fs1.cam_teleop import *
from fs1.gripper_client import *
from fs1.gripper_control import *
from fs1.joints_control import *
from fs1.keyboard_teleop import *
from fs1.servo_adapter import *
from fs1.start_hand_node import *
from fs1.vision_node import *



def test_a():
    rclpy.init(args=None)
    l = [SensorNode(),
         VisionNode(),
         CamTeleop(),
         GripperClient(),
         gripperControl()]
    for node in l:
        node.destroy_node()
    s = StartHandNode()
    msg = String()
    msg.data = dumps({})
    s.hand_status(msg)
    s.destroy_node
    rclpy.shutdown()
    assert 1==1


