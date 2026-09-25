import json

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


class VisionNode(Node):

    def __init__(self):

        super().__init__("vision_node")

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

        self.bridge = CvBridge()

        self.camera = cv2.VideoCapture(0)

        self.detector = create_detector()

        self.frame_count = 0

        self.timer = self.create_timer(
            0.03,
            self.process_frame
        )

        self.get_logger().info(
            "Vision node started."
        )

    def process_frame(self):

        ret, frame = self.camera.read()

        if not ret:

            self.get_logger().error(
                "Erro ao capturar imagem da câmera."
            )

            return

        self.frame_count += 1

        if self.frame_count % 10 != 0:

            return

        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8"
        )

        self.image_publisher.publish(
            image_msg
        )

        markers = detect_markers(
            self.detector,
            frame
        )

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

                color, percentages = (
                    detect_cube_color(
                        roi_image
                    )
                )

                if color is not None:

                    shelf_state[
                        position - 1
                    ]["occupied"] = True

                    shelf_state[
                        position - 1
                    ]["color"] = color

                # cv2.imshow(
                #     f"Cube ROI {marker_id}",
                #     roi_image
                # )

            cv2.circle(
                frame,
                marker_center,
                6,
                (0, 255, 0),
                -1
            )

            cv2.circle(
                frame,
                cube_center,
                8,
                (255, 0, 255),
                -1
            )

            cv2.rectangle(
                frame,
                (x_min_roi, y_min_roi),
                (x_max_roi, y_max_roi),
                (255, 255, 0),
                2
            )

        self.publish_shelf_state(
            shelf_state
        )

        # cv2.imshow("Vision Node", frame)
        # cv2.waitKey(1)

    def publish_shelf_state(
        self,
        shelf_state
    ):

        msg = String()

        msg.data = json.dumps(
            shelf_state
        )

        self.publisher.publish(
            msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = VisionNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.camera.release()

        cv2.destroyAllWindows()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()