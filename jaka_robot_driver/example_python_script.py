#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class ExamplePythonNode(Node):
    def __init__(self):
        super().__init__('example_python_node')
        
        # 创建发布者
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)
        
        # 创建定时器，每秒发布一次消息
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('示例Python节点已启动')
    
    def timer_callback(self):
        msg = String()
        msg.data = f'来自Python节点的消息: {self.get_clock().now().nanoseconds}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布消息: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    node = ExamplePythonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main() 