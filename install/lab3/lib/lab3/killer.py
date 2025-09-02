#!/usr/bin/python3

from lab3.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import math

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from turtlesim.srv import Kill
from controller_interfaces.srv import SetParam

class KillerNode(Node):
    def __init__(self):
        super().__init__('killer_node')
        
        # Initialize core attributes
        self.name_space = self.get_namespace()
        
        # Pose tracking variables
        self.robot_pose1 = None  # Target turtle pose
        self.robot_pose2 = None  # Own pose
        
        # Required controller parameters - DO NOT CHANGE THESE NAMES
        self.kp_linear = 0.1
        self.kp_angular = 10.0
        
        # Speed limits
        self.max_linear = 10.0
        self.max_angular = 20.0
        
        # Status flags
        self.is_kill = False
        self.can_eat = False
        
        # Get ROS parameters
        self.declare_parameter('sampling_frequency', 100.0)
        self.declare_parameter('kill_turtle', "eater_turtle")
        
        self.freq = self.get_parameter('sampling_frequency').value
        self.kill_turtle_name = self.get_parameter('kill_turtle').value
        
        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.name_space}/cmd_vel', 10)
        
        # Setup subscriptions for pose tracking
        self.target_pose_sub = self.create_subscription(
            Pose, 
            f'/{self.kill_turtle_name}/pose', 
            self.pose1_callback, 
            10
        )
        
        self.own_pose_sub = self.create_subscription(
            Pose, 
            f'{self.name_space}/pose', 
            self.pose2_callback, 
            10
        )
        
        # Subscribe to eat status
        self.eat_status_sub = self.create_subscription(
            Bool, 
            f'/{self.kill_turtle_name}/eat_status', 
            self.eat_status_callback, 
            10
        )
        
        # Service for parameter updates
        self.param_service = self.create_service(
            SetParam, 
            f'{self.name_space}/set_controller_param', 
            self.set_controller_param_callback
        )
        
        # Client for turtle elimination
        self.kill_client = self.create_client(Kill, '/remove_turtle')
        
        # Main control timer
        self.control_timer = self.create_timer(1/self.freq, self.timer_callback)
        
        self.get_logger().info(f"killer start with ns {self.name_space} and freq at {self.freq} must kill {self.kill_turtle_name}")

    def set_controller_param_callback(self, request, response):
        """Update controller gains"""
        self.kp_linear = request.kp_linear.data
        self.kp_angular = request.kp_angular.data
        self.get_logger().info(f"Current: kp_linear {self.kp_linear} kp_angular {self.kp_angular}")
        return response
    
    def kill_turtle(self):
        """Execute turtle elimination"""
        elimination_request = Kill.Request()
        elimination_request.name = self.kill_turtle_name
        self.get_logger().info(f"Im killing {self.kill_turtle_name}")
        self.kill_client.call_async(elimination_request)
        self.is_kill = True

    def pose1_callback(self, msg):
        """Handle target turtle pose updates"""
        self.robot_pose1 = [msg.x, msg.y, msg.theta]

    def pose2_callback(self, msg):
        """Handle own pose updates"""
        self.robot_pose2 = [msg.x, msg.y, msg.theta]
    
    def eat_status_callback(self, msg):
        """Monitor target eating capability"""
        self.can_eat = msg.data
    
    def pub_vel(self, vx, wz):
        """Publish velocity commands"""
        velocity_cmd = Twist()
        velocity_cmd.linear.x = vx
        velocity_cmd.angular.z = wz
        self.cmd_vel_pub.publish(velocity_cmd)
    
    def compute_pursuit_control(self):
        """Calculate control commands for pursuit"""
        if self.robot_pose1 is None or self.robot_pose2 is None:
            return 0.0, 0.0, float('inf')
        
        # Position difference calculation
        dx = self.robot_pose1[0] - self.robot_pose2[0]
        dy = self.robot_pose1[1] - self.robot_pose2[1]
        target_distance = math.sqrt(dx**2 + dy**2)
        
        # Angle calculations
        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - self.robot_pose2[2]
        normalized_heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Control law application
        linear_velocity = min(self.max_linear, max(-self.max_linear, target_distance * self.kp_linear))
        angular_velocity = min(self.max_angular, max(-self.max_angular, normalized_heading_error * self.kp_angular))
        
        return linear_velocity, angular_velocity, target_distance
    
    def timer_callback(self):
        """Main control loop"""
        # Stop if kill completed
        if self.is_kill:
            self.pub_vel(0.0, 0.0)
            return
        
        # Check pose availability
        if self.robot_pose1 is None or self.robot_pose2 is None:
            self.get_logger().info("Wait for pose")
            self.pub_vel(0.0, 0.0)
            return
        
        # PROPER GAME LOGIC: Only pursue after eater has eaten all max pizzas
        if not self.can_eat:
            self.pub_vel(0.0, 0.0)
            self.get_logger().info("WAITING: Eater hasn't finished eating all pizzas yet", throttle_duration_sec=2.0)
            return
        
        # Start pursuit when eater has finished eating all pizzas
        linear_vel, angular_vel, distance = self.compute_pursuit_control()
        self.pub_vel(linear_vel, angular_vel)
        self.get_logger().info(f"PURSUING: linear={linear_vel:.2f}, angular={angular_vel:.2f}, distance={distance:.2f}")
        
        # Execute kill when close enough
        if distance < 0.5:
            self.kill_turtle()
            self.pub_vel(0.0, 0.0)
            self.can_eat = False
            return
    


def main(args=None):
    rclpy.init(args=args)
    node = KillerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()