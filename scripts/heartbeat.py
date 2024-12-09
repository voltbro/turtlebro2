#!/usr/bin/env python3
import sys

import rclpy
import os, time, sys
from rclpy.node import Node

from geometry_msgs.msg import Pose
from pathlib import Path

ROS_DOMAIN_ID :int = int(os.environ.get('ROS_DOMAIN_ID',0))

class HeartbeatNode(Node):

    pose_time :float = time.time()

    def __init__(self):
        super().__init__('heartbeat_node')
        Path('/home/pi/.ros/.microros_heartbeat').touch() 
                
        self.pose_subscription = self.create_subscription(
            Pose,
            '/pose',
            self.pose_callback,
            10)
        
        self.get_logger().info("Init Heartbit Node")        
   

    def pose_callback(self, msg: Pose):
        Path('/home/pi/.ros/.microros_heartbeat').touch()

def main(args=None):

    rclpy.init(domain_id=ROS_DOMAIN_ID)

    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    

    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
