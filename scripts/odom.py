#!/usr/bin/env python3

import rclpy
import os
import math
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Pose2D, TransformStamped, Quaternion, PoseWithCovariance, TwistWithCovariance
from tf2_ros import TransformBroadcaster


ROS_DOMAIN_ID :int = int(os.environ.get('ROS_DOMAIN_ID',0))

class OdometryPublisher(Node):

    pose_msg: Pose = Pose()
    imu_msg: Imu = Imu()

    def __init__(self):
        super().__init__('odometry_publisher')

        self.pose2d_publisher = self.create_publisher(Pose2D, 'pose2d', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.simple_odom_timer = self.create_timer(timer_period_sec = 0.25, 
                                                   callback = self.simple_odom_publisher)
        
        self.pose_subscription = self.create_subscription(
            Pose,
            '/pose',
            self.pose_callback,
            10)
   
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        
        self.get_logger().info("Init Odometry republish")        
        # self.pose_subscription  # prevent unused variable warning

    def imu_callback(self, msg: Imu):
        self.imu_msg = msg        

    def pose_callback(self, msg: Pose):

        self.get_logger().debug('I heard: "%s"' % msg.position.x)
        stamp = self.get_clock().now().to_msg()
        self.pose_msg = msg

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(t)


        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose = PoseWithCovariance()
        odom.pose.pose.position = msg.position
        odom.pose.pose.orientation = msg.orientation

        odom.twist = TwistWithCovariance()
        odom.twist.twist.angular = self.imu_msg.angular_velocity
        # no republish linear velocity

        self.odom_publisher.publish(odom)



    def simple_odom_publisher(self):
        pose2d = Pose2D()
        pose2d.x = self.pose_msg.position.x
        pose2d.y = self.pose_msg.position.y
        pose2d.theta = self.quaternion_to_theta(self.pose_msg.orientation)
        self.pose2d_publisher.publish(pose2d)

    def quaternion_to_theta(self, orientation: Quaternion):

        t1 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        t2 = +1.0 - 2.0 * (orientation.y ** 2 + orientation.z**2)

        return math.atan2(t1, t2)        


def main(args=None):

    rclpy.init(domain_id=ROS_DOMAIN_ID)

    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    

    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

