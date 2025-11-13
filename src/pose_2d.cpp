#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Pose2DPublisher : public rclcpp::Node
{
  public:
    Pose2DPublisher()
    : Node("pose2d_publisher")
    {
      RCLCPP_INFO(this->get_logger(), "Starting pose2d_publisher CPP node");

      auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
      qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
      qos.durability(rclcpp::DurabilityPolicy::Volatile);

      pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
          "/pose", qos, 
          std::bind(&Pose2DPublisher::pose_callback, this, _1));

      pose2d_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/pose2d", qos);
            
      timer_ = this->create_wall_timer(
        250ms, std::bind(&Pose2DPublisher::publish_pose2d_callback, this));

    }

  private:
    void pose_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg) 
    {
      RCLCPP_DEBUG(this->get_logger(), "I heard pose.x: '%f'", msg->position.x);
      pose_msg = msg;
    }

    void publish_pose2d_callback()
    {
      if (pose_msg != nullptr){
        RCLCPP_DEBUG(this->get_logger(), "Pose2d pub.x: '%f'", pose_msg->position.x);

        geometry_msgs::msg::Pose2D pose;
        pose.x = pose_msg->position.x;
        pose.y = pose_msg->position.y;
        pose.theta = this->quaternion_to_theta(pose_msg->orientation);

        RCLCPP_DEBUG(
          this->get_logger(),
          "Publishing pose2d (x: %.4f, y: %.4f, theta: %.4f)",
          pose.x,
          pose.y,
          pose.theta);

        pose2d_publisher_->publish(pose);
      } else {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Skipping pose2d publish: pose_msg not received yet");
      }
    }     

    float quaternion_to_theta(const geometry_msgs::msg::Quaternion& orientation){

      const bool finite =
        std::isfinite(orientation.w) &&
        std::isfinite(orientation.x) &&
        std::isfinite(orientation.y) &&
        std::isfinite(orientation.z);

      if (!finite){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Received non-finite quaternion, reusing last valid yaw");
        return last_valid_yaw_;
      }

      const double norm_sq =
        orientation.w * orientation.w +
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z;

      if (norm_sq < 1e-9){
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Received near-zero quaternion, reusing last valid yaw");
        return last_valid_yaw_;
      }

      const double inv_norm = 1.0 / std::sqrt(norm_sq);
      const double qw = orientation.w * inv_norm;
      const double qx = orientation.x * inv_norm;
      const double qy = orientation.y * inv_norm;
      const double qz = orientation.z * inv_norm;

      const double t1 = 2.0 * (qw * qz + qx * qy);
      const double t2 = 1.0 - 2.0 * (qy * qy + qz * qz);

      last_valid_yaw_ = static_cast<float>(std::atan2(t1, t2));
      RCLCPP_DEBUG(
        this->get_logger(),
        "Computed yaw from quaternion (w: %.4f, x: %.4f, y: %.4f, z: %.4f) -> theta: %.4f",
        qw, qx, qy, qz, last_valid_yaw_);
      return last_valid_yaw_;
    }

  
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_publisher_;

    std::shared_ptr<geometry_msgs::msg::Pose> pose_msg;
    float last_valid_yaw_ {0.0f};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pose2DPublisher>());
  rclcpp::shutdown();
  return 0;
}

