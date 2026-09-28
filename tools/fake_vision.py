#!/usr/bin/env python3
"""
Fake vision publisher for integration tests without camera/robot.

Publishes the same topics/formats as ``fs1.my_node``:

* ``/shelf_state``: JSON list of ``{"position", "occupied", "color"}``.
* ``/camera/image``: raw ``bgr8`` frame (a moving gradient with a counter).

Usage:
    python3 tools/fake_vision.py --occupied 2:green 5:blue
    python3 tools/fake_vision.py            # empty shelf
"""

import argparse
import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FakeVision(Node):
    """Publish a fixed shelf inventory and a synthetic camera stream."""

    def __init__(self, occupied):
        """Create the publishers; ``occupied`` maps slot position -> color."""
        super().__init__('fake_vision')
        self.occupied = occupied
        self.shelf_pub = self.create_publisher(String, '/shelf_state', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image', 10)
        self.tick = 0
        self.create_timer(0.3, self.publish)

    def publish(self):
        """Publish one inventory message and one frame."""
        self.tick += 1
        shelf = [
            {
                'position': position,
                'occupied': position in self.occupied,
                'color': self.occupied.get(position),
            }
            for position in range(1, 9)
        ]
        self.shelf_pub.publish(String(data=json.dumps(shelf)))

        height, width = 240, 320
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:, :, 0] = np.linspace(0, 255, width, dtype=np.uint8)[None, :]
        frame[:, :, 1] = (self.tick * 8) % 256
        frame[:, :, 2] = np.linspace(255, 0, height, dtype=np.uint8)[:, None]

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fake_camera'
        msg.height, msg.width = height, width
        msg.encoding = 'bgr8'
        msg.step = width * 3
        msg.data = frame.tobytes()
        self.image_pub.publish(msg)


def main():
    """Parse ``--occupied 2:green ...`` and spin the node."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--occupied', nargs='*', default=[],
                        help='pairs POSITION:COLOR, e.g. 2:green 5:blue')
    args = parser.parse_args()
    occupied = {
        int(item.split(':')[0]): item.split(':')[1] for item in args.occupied
    }

    rclpy.init()
    node = FakeVision(occupied)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
