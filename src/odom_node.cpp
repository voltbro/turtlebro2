#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class OdometryPublisher : public rclcpp::Node
{
  public:
    OdometryPublisher()
    : Node("odometry_publisher")
    {
      pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/pose", 10, std::bind(&OdometryPublisher::pose_callback, this, _1));

      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&OdometryPublisher::imu_callback, this, _1));

      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_cpp", 10);

    }

  private:
    void pose_callback(const geometry_msgs::msg::Pose & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard ODOM.x: '%f'", msg.position.x);

        auto odom = nav_msgs::msg::Odometry();
        // odom.header.stamp = stamp
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_footprint";

        odom_publisher_->publish(odom);

        // odom.pose = PoseWithCovariance()
        // odom.pose.pose.position = msg.position
        // odom.pose.pose.orientation = msg.orientation
    }

    void imu_callback(const sensor_msgs::msg::Imu & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard  IMU.x: '%f'", msg.angular_velocity.x);
    }    

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}