#!/usr/bin/env python3

import rclpy
import os
import math
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, TransformStamped
from tf2_ros import TransformBroadcaster

ROS_DOMAIN_ID :int = int(os.environ.get('ROS_DOMAIN_ID',0))

class SimpleOdometry(Node):

    msg: Odometry = Odometry()

    def __init__(self):
        super().__init__('simple_odom_republish')

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.publisher = self.create_publisher(Pose2D, 'odom2d', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.simple_odom_timer = self.create_timer(timer_period_sec = 0.25, 
                                                   callback = self.simple_odom_publisher)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            qos_profile= qos_policy)
        self.get_logger().info("Init 2D Odometry republish")
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Odometry):

        self.msg = msg

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        self.get_logger().debug('I heard: "%s"' % msg.pose.pose.position.x)

    def simple_odom_publisher(self):
        pose2d = Pose2D()
        pose2d.x = self.msg.pose.pose.position.x
        pose2d.y = self.msg.pose.pose.position.y
        pose2d.theta = self.quaternion_to_theta(self.msg)
        self.publisher.publish(pose2d)

    def quaternion_to_theta(self, odom: Odometry):

        t1 = +2.0 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y)
        t2 = +1.0 - 2.0 * (odom.pose.pose.orientation.y ** 2 + odom.pose.pose.orientation.z**2)

        return math.atan2(t1, t2)        


def main(args=None):

    rclpy.init(domain_id=ROS_DOMAIN_ID)

    node = SimpleOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    

    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

