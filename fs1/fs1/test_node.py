import json

import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fs1.marker import (
    create_detector,
    detect_markers
)

from fs1.detector import (
    calculate_cube_roi,
    detect_cube_color,
    create_shelf_state
)
print(cv2)

class TestNode(Node):

    def __init__(self):

        super().__init__("test_node")
        self.test_status = self.create_subscription(String,"/hand_status",self.hand_status,10)
        self.publisher = self.create_publisher(String, "/switchHandDetection", 10)
        self.get_logger().info("Test node started.")
        self.activate_handNode()
    def hand_status(self,msg:String):
        d=json.loads(msg.data)
        self.get_logger().info("reciving")
        self.get_logger().info(json.dumps(d,indent=0))
    def activate_handNode(self):
        d = {"frame_jump":2 }
        msg =String()
        msg.data = json.dumps(d)
        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()