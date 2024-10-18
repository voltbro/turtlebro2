#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

using std::placeholders::_1;

#define RAD2DEG(x) ((x)*180./M_PI)

class LidarRepublisher : public rclcpp::Node
{
  public:
    LidarRepublisher()
    : Node("lidar_republisher")
    {
      RCLCPP_INFO(this->get_logger(), "Starting lidar_republisher CPP node");

      lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan_s1", rclcpp::SensorDataQoS(),
                        std::bind(&LidarRepublisher::lidar_callback, this, _1));      


      lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    }
  private:
    void lidar_callback(std::shared_ptr<sensor_msgs::msg::LaserScan> scan) 
    {
      RCLCPP_DEBUG(this->get_logger(), "Get lidar data time: '%i'", scan->header.stamp.sec);
      int count = scan->scan_time / scan->time_increment;

      for (int i = 0; i < count; i++) {        
        // float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        auto angle = scan->angle_min + scan->angle_increment * i;
        // 36 to 45 degrees
        if (angle >= 0.628319 && angle <= 0.785398){
          scan->ranges[i] = INFINITY;
        } 
      }

      lidar_publisher_->publish(*scan);
    }    

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRepublisher>());
  rclcpp::shutdown();
  return 0;
}
