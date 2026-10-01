from json import dumps

import pytest
from std_msgs.msg import String
from fs1.cam_teleop import *
from fs1.gripper_client import *
from fs1.gripper_control import *
from fs1.servo_adapter import *
from fs1.vision_node import *

#Temp file for files without specific tests
def test_a():
    rclpy.init(args=None)
    l = [VisionNode(),
         CamTeleop(),
         GripperClient(),
         gripperControl()]
    for node in l:
        rclpy.spin_once(node,timeout_sec=1.5)
        node.destroy_node()
    rclpy.shutdown()
    assert 1==1


