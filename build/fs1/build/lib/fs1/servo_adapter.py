"""
A ROS 2 node that bridges TwistStamped commands to MoveIt Servo.

It handles the robot startup sequence (escaping singularities by switching to 
JOINT_JOG, executing a trajectory, and then enabling TWIST mode) to ensure 
safe teleoperation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger

class ServoAdapter(Node):
    """
    Interface node for controlling MoveIt Servo via TwistStamped messages.
    
    Attributes:
        trajectory_pub (Publisher): Publishes the initial trajectory.
        servo_pub (Publisher): Publishes TwistStamped commands to Servo.
        cmd_vel_sub (Subscription): Subscribes to incoming TwistStamped commands.
        cli (Client): Service client to change Servo command types.
        startup_completed (bool): Indicates if the robot is ready to receive commands.
    """

    def __init__(self):
        """Initializes publishers, subscribers, and timers for the interface."""
        super().__init__('servo_adapter')
        
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.servo_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.frame_id = 'base_link' 
        
        self.startup_completed = False
        
        # Starts a loop that will check if the service is available before beginning
        self.startup_timer = self.create_timer(1.0, self.check_service_and_start)

    def check_service_and_start(self):
        """
        Step 1: Waits for the Servo service to be ready without blocking the ROS executor.
        Once ready, requests a switch to JOINT_JOG to free the trajectory controller.
        """
        if not self.cli.service_is_ready():
            self.get_logger().info('Waiting for Servo switch_command_type service...')
            return  
            
        self.startup_timer.cancel()
        self.get_logger().info('Service found. Requesting JOINT_JOG mode to allow trajectory execution...')
        
        req = Trigger.Request()
        
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_joint_jog_activated)

    def on_joint_jog_activated(self, future):
        """
        Step 2: Triggered when the robot successfully enters JOINT_JOG mode.
        Publishes the initial trajectory and starts a timer to wait for its completion.
        """
        try:
            future.result()
            self.get_logger().info('Switched to JOINT_JOG. Sending initial trajectory...')
            
            msg = JointTrajectory()
            msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            
            point = JointTrajectoryPoint()
            point.positions = [0.1, 0.1, 1.5, 0.01, 0.5, 0.01] 
            
            duration = Duration()
            duration.sec = 3
            duration.nanosec = 0
            point.time_from_start = duration
            
            msg.points = [point]
            self.trajectory_pub.publish(msg)
            
            # Wait 3.5 seconds for the physical robot to reach the position
            self.get_logger().info('Waiting 3.5s for the trajectory to complete...')
            self.trajectory_wait_timer = self.create_timer(3.5, self.request_twist_mode)
            
        except Exception as e:
            self.get_logger().error(f'Failed to switch to JOINT_JOG: {e}')

    def request_twist_mode(self):
        """
        Step 3: Triggered after the trajectory is finished. 
        Requests the Servo to switch back to TWIST mode for teleoperation.
        """
        self.trajectory_wait_timer.cancel() 
        self.get_logger().info('Trajectory finished. Requesting TWIST mode...')
        
        req = Trigger.Request()
        
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_twist_activated)

    def on_twist_activated(self, future):
        """
        Step 4: Triggered when the robot successfully enters TWIST mode.
        Enables the teleoperation pipeline.
        """
        try:
            response = future.result()
            self.get_logger().info(f'Twist mode activated successfully! Success: {response.success}')
            self.startup_completed = True 
            self.get_logger().info('*** ROBOT READY FOR TELEOPERATION ***')
        except Exception as e:
            self.get_logger().error(f'Failed to activate Twist mode: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for incoming Twist messages.
        
        Args:
            msg (Twist): The velocity command from the teleop node.
        """
        # Only forward commands if the entire startup sequence is finished
        if not self.startup_completed:
            return
            
        # Update timestamp to current time before forwarding to Servo
        new_msg = TwistStamped()
        new_msg.twist = msg
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = self.frame_id
        
        self.servo_pub.publish(new_msg)

def main(args=None):
    """Entry point for the Servo Interface node."""
    rclpy.init(args=args)
    node = ServoAdapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()