#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from jaka_robot_interfaces.msg import RobotStateDual
from std_msgs.msg import String
import time

class JakaStatusMonitor(Node):
    def __init__(self):
        super().__init__('jaka_status_monitor')
        
        # 订阅机器人状态
        self.subscription = self.create_subscription(
            RobotStateDual,
            'robot_state_dual',
            self.robot_state_callback,
            10
        )
        
        # 发布状态信息
        self.status_publisher = self.create_publisher(
            String,
            'jaka_status',
            10
        )
        
        # 创建定时器，每5秒发布一次状态摘要
        self.timer = self.create_timer(5.0, self.status_timer_callback)
        
        self.get_logger().info('JAKA状态监控节点已启动')
        
        # 状态变量
        self.last_left_joint = None
        self.last_right_joint = None
        self.message_count = 0
    
    def robot_state_callback(self, msg):
        """处理机器人状态消息"""
        self.message_count += 1
        
        # 检查关节角度变化
        left_changed = self.last_left_joint != msg.joint_pos_left.joint_values
        right_changed = self.last_right_joint != msg.joint_pos_right.joint_values
        
        if left_changed or right_changed:
            self.get_logger().info(f'关节角度发生变化 - 消息计数: {self.message_count}')
            
            # 记录新的关节角度
            self.last_left_joint = msg.joint_pos_left.joint_values.copy()
            self.last_right_joint = msg.joint_pos_right.joint_values.copy()
    
    def status_timer_callback(self):
        """定时发布状态摘要"""
        status_msg = String()
        status_msg.data = f'JAKA状态监控 - 已接收消息: {self.message_count}'
        
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f'发布状态摘要: {status_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    node = JakaStatusMonitor()
    
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