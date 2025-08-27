#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class TurtlesimPosePublisher(Node):
    def __init__(self):
        super().__init__('odom_pub')

        self.transform_publisher = TransformBroadcaster(self)
        
        self.turtle1_pose = np.array([0.0, 0.0, 0.0])
        self.turtle2_pose = np.array([0.0, 0.0, 0.0])

        self.create_subscription(Pose, '/turtle2/pose', self.handle_turtle2_pose, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.handle_turtle1_pose, 10)
        
        self.turtle1_odom_publisher = self.create_publisher(Odometry, '/odom1', 10)
        self.turtle2_odom_publisher = self.create_publisher(Odometry, '/odom2', 10)

    def handle_turtle2_pose(self, pose_message):
        if pose_message.x >= 0 and pose_message.y >= 0:
            angle_diff = abs(pose_message.theta - self.turtle2_pose[2])
            position_diff = np.sqrt((pose_message.x - self.turtle2_pose[0])**2 + (pose_message.y - self.turtle2_pose[1])**2)
            
            if position_diff > 0.01 or angle_diff > 0.02:
                self.turtle2_pose[0] = pose_message.x
                self.turtle2_pose[1] = pose_message.y
                self.turtle2_pose[2] = pose_message.theta
                self.get_logger().debug(f'Turtle2 pose updated: x={pose_message.x:.2f}, y={pose_message.y:.2f}, theta={pose_message.theta:.2f}')
                self.publish_odometry('turtle2', self.turtle2_pose)
    
    def handle_turtle1_pose(self, pose_message):
        if pose_message.x >= 0 and pose_message.y >= 0:
            distance_moved = np.sqrt((pose_message.x - self.turtle1_pose[0])**2 + (pose_message.y - self.turtle1_pose[1])**2)
            if distance_moved > 0.01:
                self.turtle1_pose[0] = pose_message.x
                self.turtle1_pose[1] = pose_message.y
                self.turtle1_pose[2] = pose_message.theta
                self.publish_odometry('turtle1', self.turtle1_pose)

    
    def publish_odometry(self, frame_id, turtle_pose):
        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = frame_id
        self.get_logger().debug(f'Publishing odometry for {frame_id}')
        
        odometry.pose.pose.position.x = turtle_pose[0] - 5.44
        odometry.pose.pose.position.y = turtle_pose[1] - 5.44

        quaternion = quaternion_from_euler(0, 0, turtle_pose[2])
        odometry.pose.pose.orientation.x = quaternion[0]
        odometry.pose.pose.orientation.y = quaternion[1]
        odometry.pose.pose.orientation.z = quaternion[2]
        odometry.pose.pose.orientation.w = quaternion[3]

        if frame_id == 'turtle1':
            self.turtle1_odom_publisher.publish(odometry)
            self.get_logger().info(f'Turtle1 odometry published: pos=({turtle_pose[0]-5.44:.2f}, {turtle_pose[1]-5.44:.2f})')
        else:
            self.turtle2_odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = frame_id
        transform.transform.translation.x = turtle_pose[0] - 5.44
        transform.transform.translation.y = turtle_pose[1] - 5.44
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        self.transform_publisher.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = TurtlesimPosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()