"""Small helper node that activates hand detection on startup (manual test tool)."""

import json
from time import sleep
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fs1.marker import create_detector,detect_markers
from fs1.detector import calculate_cube_roi,detect_cube_color,create_shelf_state

#print(cv2.__version__)

class StartHandNode(Node):
    '''Usado somente para ativar o detector de mão em hand_node e testar o retorno dele'''    """Waits for the stack to settle, then switches on hand detection."""

    def __init__(self,print_status = False):
        """Wait 5s for other nodes to be ready, then activate hand detection."""
        sleep(5)
        super().__init__("start_hand_node")
        if print_status: self.test_status = self.create_subscription(String,"/hand_status",self.hand_status,10)
        self.publisher = self.create_publisher(String, "/switchHandDetection", 10)
        self.get_logger().info("Test node started.")
        self.activate_handNode()
    def hand_status(self,msg:String):
        """Parse an incoming ``/hand_status`` message (no-op, kept for manual testing)."""
        d=json.loads(msg.data)
        self.get_logger().info("reciving")
        self.get_logger().info(json.dumps(d,indent=0))
    def activate_handNode(self):
        """Publish the ``/switchHandDetection`` command that turns hand_node on."""
        d = {"frame_jump":3 }
        msg =String()
        msg.data = json.dumps(d)
        self.publisher.publish(msg)



def main(args=None):# pragma: no cover
    """Entry point: run StartHandNode once at startup."""
    rclpy.init(args=args)
    node = StartHandNode()

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