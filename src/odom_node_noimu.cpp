#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
      qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
      qos.durability(rclcpp::DurabilityPolicy::Volatile);

      pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
          "/pose", qos, 
          std::bind(&OdometryPublisher::pose_callback, this, _1));

      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);

    }

  private:
    void pose_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg) 
    {
      RCLCPP_DEBUG(this->get_logger(), "I heard ODOM.x: '%f'", msg->position.x);

      rclcpp::Time now = this->get_clock()->now();

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = now;
      odom.header.frame_id = "odom";
      odom.child_frame_id  = "base_footprint";

      odom.pose.pose.position = msg->position;
      odom.pose.pose.orientation = msg->orientation;

      RCLCPP_DEBUG(
        this->get_logger(),
        "Publishing odom pose (x: %.4f, y: %.4f, z: %.4f)",
        msg->position.x,
        msg->position.y,
        msg->position.z);

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

  
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}