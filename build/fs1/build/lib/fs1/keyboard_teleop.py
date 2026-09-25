"""
A ROS 2 node for keyboard teleoperation.

Captures keyboard inputs in raw mode from the terminal and publishes 
standard geometry_msgs/Twist messages to the '/cmd_vel' topic.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):
    """
    A standalone ROS 2 node that reads keystrokes and publishes Twist messages.
    
    Attributes:
        publisher_ (Publisher): Publishes twist commands to '/cmd_vel'.
        speed (float): Current translation speed in meters per second.
        target_y (float): Target velocity along the Y-axis.
        target_z (float): Target velocity along the Z-axis.
    """

    def __init__(self):
        """Initializes the teleop node and its publisher."""
        super().__init__('keyboard_teleop')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.speed = 1.0
        self.target_y = 0.0
        self.target_z = 0.0

    def keyboard_loop(self):
        """
        Main blocking loop that captures keyboard input.
        
        Sets the terminal to raw mode. Evaluates pressed keys and updates 
        the Twist message. If no movement key is pressed, it publishes zeros 
        to halt the robot.
        """
        msg = """
            Teleop Node Started!
            ---------------------------
            Movement (Hold down):
            w: +Z (Up)
            s: -Z (Down)
            a: +Y (Left)
            d: -Y (Right)
            
            Speed:
            q: Increase speed (x1.1)
            e: Decrease speed (x0.9)
            
            Press Ctrl+C to exit.
            ---------------------------
        """
        self.get_logger().info(msg)
        print(msg)
        settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                
                if rlist:
                    key = sys.stdin.read(1)
                else:
                    key = ''
                
                if key == '\x03':  # Ctrl+C
                    break
                
                # Logic map
                if key == 'w':
                    self.target_z = 0.01 * self.speed
                    self.target_y = 0.0
                elif key == 's':
                    self.target_z = -0.01 * self.speed
                    self.target_y = 0.0
                elif key == 'a':
                    self.target_y = 0.01 * self.speed
                    self.target_z = 0.0
                elif key == 'd':
                    self.target_y = -0.01 * self.speed
                    self.target_z = 0.0
                elif key == 'q':
                    self.speed *= 1.1
                    sys.stdout.write(f"\rCurrent speed: {self.speed:.4f} m/s \033[K")
                    sys.stdout.flush()
                elif key == 'e':
                    self.speed *= 0.9
                    self.speed = max(0.001, self.speed)
                    sys.stdout.write(f"\rCurrent speed: {self.speed:.4f} m/s \033[K")
                    sys.stdout.flush()
                else:
                    # Stop if no key is held
                    self.target_y = 0.0
                    self.target_z = 0.0

                # Construct and publish the Twist message
                twist = Twist()
                twist.linear.y = self.target_y
                twist.linear.z = self.target_z
                self.publisher_.publish(twist)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    """Entry point for the Keyboard Teleop node."""
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        node.keyboard_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()