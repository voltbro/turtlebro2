#!/usr/bin/env python3

import rclpy
import os
import math
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


ROS_DOMAIN_ID :int = int(os.environ.get('ROS_DOMAIN_ID',0))

class SimpleOdometry(Node):

    def __init__(self):
        super().__init__('simple_odom_republish')

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.publisher = self.create_publisher(Pose2D, 'odom2d', 10)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            qos_profile= qos_policy)
        self.get_logger().info("Init 2D Odometry republish")
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Odometry):

        pose2d = Pose2D()
        pose2d.x = msg.pose.pose.position.x
        pose2d.y = msg.pose.pose.position.y
        pose2d.theta = self.quaternion_to_theta(msg)
        self.publisher.publish(pose2d)

        self.get_logger().debug('I heard: "%s"' % msg.pose.pose.position.x)

    def quaternion_to_theta(self, odom: Odometry):

        t1 = +2.0 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y)
        t2 = +1.0 - 2.0 * (odom.pose.pose.orientation.y ** 2 + odom.pose.pose.orientation.z**2)

        return math.atan2(t1, t2)        


def main(args=None):

    rclpy.init(domain_id=ROS_DOMAIN_ID)

    simple_subscriber = SimpleOdometry()

    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

