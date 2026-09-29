"""
Shelf/ArUco vision node: detects markers, classifies piece colors and
publishes both the shelf inventory (``/shelf_state``) and the raw camera
feed (``/camera/image``, republished as JPEG for the front-end).

Also answers on-demand cube requests: publishing anything to
``/request_cube`` makes the next processed frame pick a random occupied
slot and publish it on ``/selected_cube`` (``{"available", "position",
"color"}``). Not currently used by ``fs1.supervisor``, which does its own
persistence-checked random pick (RNF#02) straight from ``/shelf_state``.
"""

import json
import random

import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from fs1.marker import (
    create_detector,
    detect_markers
)

from fs1.detector import (
    calculate_cube_roi,
    detect_cube_color,
    create_shelf_state
)


def _parse_camera_index(value):
    """
    Turn the ``camera_index`` parameter into what ``cv2.VideoCapture`` wants.

    Accepts a plain number (``0``, ``"2"``) or a stable device path (e.g.
    ``/dev/v4l/by-id/usb-Logitech_BRIO-video-index0``, which keeps working
    after the camera is unplugged and plugged into a different USB port,
    unlike ``/dev/videoN``'s number).
    """
    text = str(value)
    return int(text) if text.lstrip('-').isdigit() else text


class VisionNode(Node):
    """Reads the effector camera, maps ArUco markers to shelf slots and colors."""

    def __init__(self):
        """Open the camera and set up the publishers, timer and parameters."""
        super().__init__("vision_node")

        # Janelas de depuração (cv2.imshow) desligadas por padrão: abrem uma
        # janela nativa na tela de quem estiver rodando o container com
        # DISPLAY configurado, o que atrapalha quem só quer ver o front-end.
        # Ligue com: ros2 run fs1 vision_node --ros-args -p debug_windows:=true
        self.declare_parameter('debug_windows', False)
        self.debug_windows = self.get_parameter('debug_windows').value

        # Índice da câmera do efetuador (2K, acoplada ao robô). No
        # laboratório ela e a webcam do operador (hand_node) são
        # dispositivos físicos diferentes; ajuste via
        # --ros-args -p camera_index:=<n> em vez de editar o código. Pode
        # ser um número (0, 1, 2...) ou um caminho estável, tipo
        # /dev/v4l/by-id/usb-<algo>-video-index0 (não muda ao trocar de
        # porta USB, diferente do número, que muda).
        self.declare_parameter('camera_index', '0')
        camera_index = _parse_camera_index(
            self.get_parameter('camera_index').value)

        self.publisher = self.create_publisher(
            String,
            "/shelf_state",
            10
        )

        self.image_publisher = self.create_publisher(
            Image,
            "/camera/image",
            10
        )

        self.selected_cube_publisher = self.create_publisher(
            String,
            "/selected_cube",
            10
        )

        self.request_cube_subscription = self.create_subscription(
            String,
            "/request_cube",
            self.request_cube_callback,
            10
        )

        self.bridge = CvBridge()

        self.camera = cv2.VideoCapture(camera_index)

        self.detector = create_detector()

        self.frame_count = 0

        self.cube_request_pending = False

        self.timer = self.create_timer(0.03, self.process_frame)

        self.get_logger().info("Vision node started.")

    def request_cube_callback(self, msg):
        """Flag a random-cube pick to run on the next processed frame."""
        self.get_logger().info( f"Solicitação de novo cubo recebida: {msg.data}" )

        self.cube_request_pending = True

    def process_frame(self):
        """
        Grab one frame, detect markers/colors and publish the shelf state.

        Runs on a 0.03 s timer; only every 10th frame is actually
        processed/published, to keep the CPU and topic rate reasonable.
        """
        ret, frame = self.camera.read()

        if not ret:

            self.get_logger().error("Erro ao capturar imagem da câmera.")

            return

        self.frame_count += 1

        if self.frame_count % 10 != 0:

            return

        image_msg = self.bridge.cv2_to_imgmsg( frame, encoding="bgr8" )

        self.image_publisher.publish( image_msg )

        markers = detect_markers( self.detector, frame )

        shelf_state = create_shelf_state()

        for marker_id, info in markers.items():

            position = info["position"]

            marker_center = info["center"]

            marker_size_px = info["size_px"]

            cube_center, roi = calculate_cube_roi(
                marker_center,
                marker_size_px
            )

            (
                x_min_roi,
                y_min_roi,
                x_max_roi,
                y_max_roi
            ) = roi

            roi_image = frame[
                y_min_roi:y_max_roi,
                x_min_roi:x_max_roi
            ]

            if roi_image.size > 0:

                color, percentages = (detect_cube_color( roi_image ))

                if color is not None:

                    shelf_state[ position - 1 ]["occupied"] = True

                    shelf_state[ position - 1 ]["color"] = color

                if self.debug_windows:
                    cv2.imshow( f"Cube ROI {marker_id}", roi_image)

            cv2.circle( frame, marker_center, 6, (0, 255, 0), -1 )
            cv2.circle( frame, cube_center, 8, (255, 0, 255), -1 )
            cv2.rectangle( frame, (x_min_roi, y_min_roi), (x_max_roi, y_max_roi), (255, 255, 0), 2 )

        self.publish_shelf_state( shelf_state )


        if self.cube_request_pending:

            self.select_random_cube( shelf_state )

            self.cube_request_pending = False

        if self.debug_windows:
            cv2.imshow("Vision Node", frame)
            cv2.waitKey(1)

    def publish_shelf_state(self, shelf_state ):
        """Publish ``shelf_state`` (list of 8 slot dicts) as JSON."""
        msg = String()
        msg.data = json.dumps( shelf_state )
        self.publisher.publish( msg )

    def select_random_cube( self, shelf_state ):
        """Publish a random occupied slot from ``shelf_state`` on ``/selected_cube``."""
        available_cubes = [ cube for cube in shelf_state if cube["occupied"] ]

        if not available_cubes:

            self.get_logger().warning("Nenhum cubo disponível para sorteio." )
            msg = String()
            msg.data = json.dumps({
                "available": False,
                "position": None,
                "color": None
            })

            self.selected_cube_publisher.publish( msg )

            return

        selected_cube = random.choice( available_cubes )
        msg = String()
        msg.data = json.dumps({
            "available": True,
            "position": selected_cube["position"],
            "color": selected_cube["color"]
        })

        self.selected_cube_publisher.publish(msg)


    def destroy_node(self):
        """Release the camera and close any debug windows before shutting down."""
        self.camera.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Entry point for the shelf/ArUco vision node."""
    rclpy.init(args=args)
    node = VisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()