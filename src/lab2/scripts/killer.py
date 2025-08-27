#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
from turtlesim.msg import Pose
from std_msgs.msg import Int64

class Killer(Node):
    def __init__(self):
        super().__init__('killer_node')

        self.mouse_location = np.array([0.0, 0.0])
        self.hunter_position = np.array([0.0, 0.0, 0.0])
        self.target_turtle_position = np.array([0.0, 0.0, 0.0])
        
        self.pizza_counter = 0
        self.pizza_threshold = 5

        self.elimination_client = self.create_client(Kill, 'remove_turtle')
        
        self.forward_gain = 11.0
        self.rotation_gain = 20.0

        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.target_pose_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.hunter_pose_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if (self.pizza_counter < self.pizza_threshold):
            self.get_logger().info(f'Waiting for pizza threshold: {self.pizza_counter}/{self.pizza_threshold}')
            self.publish_velocity(0.0, 0.0)
            return
        
        pursuit_target = self.target_turtle_position
        x_difference = pursuit_target[0] - self.hunter_position[0]
        y_difference = pursuit_target[1] - self.hunter_position[1]
        target_distance = np.sqrt(x_difference**2 + y_difference**2)
        desired_angle = np.arctan2(y_difference, x_difference)
        angular_error = desired_angle - self.hunter_position[2]
        angular_error = np.arctan2(np.sin(angular_error), np.cos(angular_error))

        forward_speed = self.forward_gain * target_distance
        rotation_speed = self.rotation_gain * angular_error
        self.publish_velocity(forward_speed, rotation_speed)

        if target_distance < 0.1 and abs(angular_error) < 0.1:
            self.get_logger().info('Target reached, initiating elimination sequence')
            self.eliminate_target()

    def publish_velocity(self, linear_speed, angular_speed):
        motion_command = Twist()
        motion_command.linear.x = linear_speed
        motion_command.angular.z = angular_speed
        self.cmd_vel_pub.publish(motion_command)

    def hunter_pose_callback(self, hunter_msg):
        if hunter_msg.x >= 0 and hunter_msg.y >= 0:
            prev_x, prev_y = self.hunter_position[0], self.hunter_position[1]
            self.hunter_position[0] = hunter_msg.x
            self.hunter_position[1] = hunter_msg.y
            self.hunter_position[2] = hunter_msg.theta
            
            distance_moved = np.sqrt((hunter_msg.x - prev_x)**2 + (hunter_msg.y - prev_y)**2)
            if distance_moved > 2.0:
                self.publish_velocity(0.0, 0.0)
    
    def target_pose_callback(self, target_msg):
        if hasattr(target_msg, 'x') and hasattr(target_msg, 'y') and hasattr(target_msg, 'theta'):
            old_distance = np.sqrt((self.target_turtle_position[0] - self.hunter_position[0])**2 + 
                                 (self.target_turtle_position[1] - self.hunter_position[1])**2)
            
            self.target_turtle_position[0] = target_msg.x
            self.target_turtle_position[1] = target_msg.y
            self.target_turtle_position[2] = target_msg.theta
            
            new_distance = np.sqrt((self.target_turtle_position[0] - self.hunter_position[0])**2 + 
                                 (self.target_turtle_position[1] - self.hunter_position[1])**2)
            
            if abs(new_distance - old_distance) > 0.5:
                self.get_logger().debug(f'Target distance changed: {old_distance:.2f} -> {new_distance:.2f}')
        else:
            self.get_logger().warning('Invalid target pose message received')

    def eliminate_target(self):
        kill_request = Kill.Request()
        kill_request.name = "turtle1"
        elimination_future = self.elimination_client.call_async(kill_request)
        elimination_future.add_done_callback(self.target_eliminated_callback)

    def target_eliminated_callback(self, completed_future):
        completed_future.result()
        self.get_logger().info('Target eliminated successfully')
        self.publish_velocity(0.0, 0.0)

    def pizza_count_callback(self, count_message):
        old_count = self.pizza_counter
        self.pizza_counter = max(0, count_message.data)
        
        if self.pizza_counter >= self.pizza_threshold and old_count < self.pizza_threshold:
            self.get_logger().info('Pizza threshold reached! Hunter mode activated')
            
        if self.pizza_counter == 0 and old_count > 0:
            self.publish_velocity(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    killer_node = Killer()
    rclpy.spin(killer_node)
    killer_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()