#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node
{
  public:
    OdometryPublisher()
    : Node("odometry_publisher")
    {
      RCLCPP_INFO(this->get_logger(), "Starting odometry_publisher CPP node");

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
          "/pose", 10, 
          std::bind(&OdometryPublisher::pose_callback, this, _1));

      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/imu", 10, 
          std::bind(&OdometryPublisher::imu_callback, this, _1));

      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      pose2d_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/pose2d", 10);
            
      timer_ = this->create_wall_timer(
        250ms, std::bind(&OdometryPublisher::publish_pose2d_callback, this));

    }

  private:
    void pose_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg) 
    {
      RCLCPP_DEBUG(this->get_logger(), "I heard ODOM.x: '%f'", msg->position.x);

      pose_msg = msg;
      rclcpp::Time now = this->get_clock()->now();

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = now;
      odom.header.frame_id = "odom";
      odom.child_frame_id  = "base_footprint";

      odom.pose.pose.position = msg->position;
      odom.pose.pose.orientation = msg->orientation;

      if (imu_msg != nullptr){
        odom.twist.twist.angular = imu_msg->angular_velocity;
      }

      odom_publisher_->publish(odom);

      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = "odom";
      t.child_frame_id  = "base_footprint";     
      t.transform.translation.x = msg->position.x;
      t.transform.translation.y = msg->position.y;
      t.transform.translation.z = 0.0;
      t.transform.rotation = msg->orientation;  
      tf_broadcaster_->sendTransform(t);

    }

    void imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg) 
    {
      RCLCPP_DEBUG(this->get_logger(), "I heard  IMU.x: '%f'", msg->angular_velocity.x);
      imu_msg = msg;
    }

    void publish_pose2d_callback()
    {
      if (pose_msg != nullptr){
        RCLCPP_DEBUG(this->get_logger(), "Pose2d pub.x: '%f'", pose_msg->position.x);

        geometry_msgs::msg::Pose2D pose;
        pose.x = pose_msg->position.x;
        pose.y = pose_msg->position.y;
        pose.theta = this->quaternion_to_theta(pose_msg->orientation);

        pose2d_publisher_->publish(pose);
      }
    }     

    float quaternion_to_theta(const geometry_msgs::msg::Quaternion& orientation){

      auto t1 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
      auto t2 = +1.0 - 2.0 * (pow(orientation.y,2) + pow(orientation.z,2));

      return atan2(t1, t2);   
    }

  
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_publisher_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<sensor_msgs::msg::Imu> imu_msg; 
    std::shared_ptr<geometry_msgs::msg::Pose> pose_msg;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}