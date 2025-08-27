#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Int64

class Eater(Node):
    def __init__(self):
        super().__init__('eater_node')

        self.mouse_position = np.array([0.0, 0.0])
        self.turtle_position = np.array([0.0, 0.0, 0.0])
        self.current_target = np.array([0.0, 0.0])
        self.evasion_target = []
        
        self.target_queue = []
        self.collected_pizzas = 0
        self.spawned_pizzas = 0
        self.pizza_limit = 5

        self.pizza_spawner = self.create_client(GivePosition, 'spawn_pizza')
        self.pizza_eater = self.create_client(Empty, '/turtle1/eat')

        self.linear_gain = 5.0
        self.angular_gain = 20.0

        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if not self.target_queue:
            if not self.evasion_target:
                self.get_logger().info('No targets available, stopping turtle')
                self.send_velocity_command(0.0, 0.0)
                return
            else:
                self.current_target = self.evasion_target
        else:
            self.current_target = self.target_queue[0]

        delta_x = self.current_target[0] - self.turtle_position[0]
        delta_y = self.current_target[1] - self.turtle_position[1]
        distance = np.sqrt(delta_x**2 + delta_y**2)
        target_angle = np.arctan2(delta_y, delta_x)
        angle_error = target_angle - self.turtle_position[2]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        linear_velocity = self.linear_gain * distance
        angular_velocity = self.angular_gain * angle_error
        self.send_velocity_command(linear_velocity, angular_velocity)

        if distance < 0.1 and abs(angle_error) < 0.1:
            self.get_logger().info(f'Reached target at ({self.current_target[0]:.2f}, {self.current_target[1]:.2f})')
            self.send_velocity_command(0.0, 0.0)
            self.consume_pizza()

    def send_velocity_command(self, linear_vel, angular_vel):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear_vel
        velocity_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(velocity_msg)

    def mouse_position_callback(self, position_msg):
        new_x, new_y = position_msg.x, position_msg.y
        
        if abs(new_x - self.mouse_position[0]) > 0.1 or abs(new_y - self.mouse_position[1]) > 0.1:
            self.mouse_position[0] = new_x
            self.mouse_position[1] = new_y
            
            if 0.5 <= new_x <= 10.5 and 0.5 <= new_y <= 10.5:
                if (self.collected_pizzas < self.pizza_limit) and (self.spawned_pizzas < self.pizza_limit):
                    self.create_pizza(new_x, new_y)
                else:
                    self.evasion_target = [new_x, new_y]

    def create_pizza(self, x_coord, y_coord):
        spawn_request = GivePosition.Request()
        spawn_request.x = x_coord
        spawn_request.y = y_coord
        self.target_queue.append([x_coord, y_coord])
        self.spawned_pizzas += 1
        self.get_logger().info(f'Spawning pizza at ({x_coord:.2f}, {y_coord:.2f}), total spawned: {self.spawned_pizzas}')
        self.pizza_spawner.call_async(spawn_request)

    def pose_callback(self, pose_msg):
        if pose_msg.x >= 0 and pose_msg.y >= 0:
            self.turtle_position[0] = pose_msg.x
            self.turtle_position[1] = pose_msg.y
            self.turtle_position[2] = pose_msg.theta
    
    def goal_pose_callback(self, goal_msg):
        try:
            adjusted_x = goal_msg.pose.position.x + 5.44
            adjusted_y = goal_msg.pose.position.y + 5.44
            
            adjusted_x = max(0.5, min(10.5, adjusted_x))
            adjusted_y = max(0.5, min(10.5, adjusted_y))
            
            if (self.collected_pizzas < self.pizza_limit) and (self.spawned_pizzas < self.pizza_limit):
                distance_to_goal = np.sqrt((adjusted_x - self.turtle_position[0])**2 + (adjusted_y - self.turtle_position[1])**2)
                if distance_to_goal > 0.5:
                    self.create_pizza(adjusted_x, adjusted_y)
            else:
                self.evasion_target = [adjusted_x, adjusted_y]
        except AttributeError as e:
            self.get_logger().warning(f'Invalid goal pose message: {e}')

    def pizza_count_callback(self, count_msg):
        if count_msg.data >= 0 and count_msg.data != self.collected_pizzas:
            self.collected_pizzas = count_msg.data
            if self.collected_pizzas >= self.pizza_limit:
                self.target_queue.clear()

    def consume_pizza(self):
        consume_request = Empty.Request()
        consume_future = self.pizza_eater.call_async(consume_request)
        consume_future.add_done_callback(self.pizza_consumed_callback)

    def pizza_consumed_callback(self, completed_future):
        try:
            result = completed_future.result()
            if self.target_queue:
                removed_target = self.target_queue.pop(0)
                self.get_logger().debug(f'Consumed pizza at ({removed_target[0]:.2f}, {removed_target[1]:.2f})')
        except Exception as e:
            self.get_logger().error(f'Failed to consume pizza: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    eater_node = Eater()
    rclpy.spin(eater_node)
    eater_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()