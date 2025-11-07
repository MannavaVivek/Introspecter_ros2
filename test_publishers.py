#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32, Float64, Bool, Header
from geometry_msgs.msg import Pose, Twist, PoseStamped, Vector3
from sensor_msgs.msg import Temperature, Imu, JointState
from nav_msgs.msg import Odometry
import math
import random


class TestPublishersNode(Node):
    def __init__(self):
        super().__init__('test_publishers_node')
        
        self.get_logger().info('Initializing Test Publishers Node with 10 publishers')
        
        # Counter for generating varying data
        self.counter = 0
        
        # Publisher 1: String messages at 1 Hz
        self.pub1 = self.create_publisher(String, '/test/string_topic', 10)
        self.timer1 = self.create_timer(1.0, self.publish_string)
        
        # Publisher 2: Int32 messages at 2 Hz
        self.pub2 = self.create_publisher(Int32, '/test/int_topic', 10)
        self.timer2 = self.create_timer(0.5, self.publish_int)
        
        # Publisher 3: Float64 messages at 5 Hz
        self.pub3 = self.create_publisher(Float64, '/test/float_topic', 10)
        self.timer3 = self.create_timer(0.2, self.publish_float)
        
        # Publisher 4: Bool messages at 0.5 Hz
        self.pub4 = self.create_publisher(Bool, '/test/bool_topic', 10)
        self.timer4 = self.create_timer(2.0, self.publish_bool)
        
        # Publisher 5: Pose messages at 10 Hz
        self.pub5 = self.create_publisher(Pose, '/test/pose_topic', 10)
        self.timer5 = self.create_timer(0.1, self.publish_pose)
        
        # Publisher 6: Twist messages at 20 Hz
        self.pub6 = self.create_publisher(Twist, '/test/twist_topic', 10)
        self.timer6 = self.create_timer(0.05, self.publish_twist)
        
        # Publisher 7: Temperature messages at 0.2 Hz
        self.pub7 = self.create_publisher(Temperature, '/test/temperature_topic', 10)
        self.timer7 = self.create_timer(5.0, self.publish_temperature)
        
        # Publisher 8: PoseStamped messages at 15 Hz
        self.pub8 = self.create_publisher(PoseStamped, '/test/pose_stamped_topic', 10)
        self.timer8 = self.create_timer(0.067, self.publish_pose_stamped)
        
        # Publisher 9: JointState messages at 30 Hz
        self.pub9 = self.create_publisher(JointState, '/test/joint_states_topic', 10)
        self.timer9 = self.create_timer(0.033, self.publish_joint_states)
        
        # Publisher 10: Odometry messages at 50 Hz
        self.pub10 = self.create_publisher(Odometry, '/test/odom_topic', 10)
        self.timer10 = self.create_timer(0.02, self.publish_odometry)
        
        self.get_logger().info('All publishers initialized successfully')
    
    def publish_string(self):
        msg = String()
        msg.data = f'Hello ROS2! Counter: {self.counter}'
        self.pub1.publish(msg)
        self.get_logger().debug(f'Published String: {msg.data}')
    
    def publish_int(self):
        msg = Int32()
        msg.data = self.counter % 100
        self.pub2.publish(msg)
        self.get_logger().debug(f'Published Int32: {msg.data}')
    
    def publish_float(self):
        msg = Float64()
        msg.data = math.sin(self.counter * 0.1) * 10.0
        self.pub3.publish(msg)
        self.get_logger().debug(f'Published Float64: {msg.data:.2f}')
    
    def publish_bool(self):
        msg = Bool()
        msg.data = (self.counter % 2 == 0)
        self.pub4.publish(msg)
        self.get_logger().debug(f'Published Bool: {msg.data}')
    
    def publish_pose(self):
        msg = Pose()
        msg.position.x = math.cos(self.counter * 0.1) * 2.0
        msg.position.y = math.sin(self.counter * 0.1) * 2.0
        msg.position.z = 0.5
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        self.pub5.publish(msg)
        self.get_logger().debug(f'Published Pose at ({msg.position.x:.2f}, {msg.position.y:.2f})')
    
    def publish_twist(self):
        msg = Twist()
        msg.linear.x = math.sin(self.counter * 0.05) * 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = math.cos(self.counter * 0.05) * 0.3
        self.pub6.publish(msg)
        self.get_logger().debug(f'Published Twist: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')
    
    def publish_temperature(self):
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.temperature = 20.0 + random.uniform(-5.0, 5.0)
        msg.variance = 0.5
        self.pub7.publish(msg)
        self.get_logger().debug(f'Published Temperature: {msg.temperature:.2f}°C')
    
    def publish_pose_stamped(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.counter * 0.01
        msg.pose.position.y = math.sin(self.counter * 0.1)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub8.publish(msg)
        self.get_logger().debug(f'Published PoseStamped in frame: {msg.header.frame_id}')
    
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        msg.position = [
            math.sin(self.counter * 0.02),
            math.cos(self.counter * 0.03),
            math.sin(self.counter * 0.04) * 0.5,
            math.cos(self.counter * 0.05) * 0.5
        ]
        msg.velocity = [0.1, 0.2, 0.15, 0.25]
        msg.effort = [1.0, 1.5, 0.8, 1.2]
        self.pub9.publish(msg)
        self.get_logger().debug(f'Published JointState with {len(msg.name)} joints')
    
    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        # Position
        msg.pose.pose.position.x = self.counter * 0.001
        msg.pose.pose.position.y = math.sin(self.counter * 0.01) * 0.5
        msg.pose.pose.position.z = 0.0
        
        # Orientation
        msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.counter * 0.01) * 0.1
        
        # Velocities
        msg.twist.twist.linear.x = 0.5
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.1
        
        self.pub10.publish(msg)
        self.get_logger().debug('Published Odometry')
        
        # Increment counter for all callbacks
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = TestPublishersNode()
    
    # Use MultiThreadedExecutor to handle all timers concurrently
    executor = MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)
    
    try:
        node.get_logger().info('Starting MultiThreaded executor with 10 threads...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

