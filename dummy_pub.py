#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import threading


class MultiTopicPublisher(Node):
    def __init__(self):
        super().__init__('dummy_pub')
        
        self.pubs_ = []
        self.timers_ = []
        self.counters = {}
        
        # Create 50 publishers and timers
        for i in range(1, 51):
            topic_name = f'chitter{i}'
            
            # Create publisher
            publisher = self.create_publisher(String, topic_name, 10)
            self.pubs_.append(publisher)
            
            # Initialize counter
            self.counters[topic_name] = 0
            
            # Create timer with different periods to simulate concurrent publishing
            # All publish at 10 Hz (0.1 second period)
            timer = self.create_timer(
                0.1,
                lambda topic=topic_name, pub=publisher: self.timer_callback(topic, pub)
            )
            self.timers_.append(timer)
            
        self.get_logger().info(f'Created {len(self.pubs_)} publishers')
        self.get_logger().info('Publishing on topics: chitter1 to chitter50')
        
    def timer_callback(self, topic_name, publisher):
        """Callback function for each timer"""
        msg = String()
        self.counters[topic_name] += 1
        msg.data = f'{topic_name}: message {self.counters[topic_name]} from thread {threading.current_thread().name}'
        publisher.publish(msg)
        
        # Log occasionally to avoid spam
        if self.counters[topic_name] % 100 == 0:
            self.get_logger().info(f'Published {self.counters[topic_name]} messages on {topic_name}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = MultiTopicPublisher()
    
    # Create a multi-threaded executor with enough threads
    # Using 50 threads so each topic can publish concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info('Starting multi-threaded executor with 50 threads...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

