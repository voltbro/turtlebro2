/*

ros2 run turtlebro lidar_republish --ros-args --log-level DEBUG -p strip_angle_start:=1 -p strip_angle_stop:=2
*/
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

using std::placeholders::_1;

#define RAD2DEG(x) ((x) * 180. / M_PI)
#define DEG2RAD(x) ((x * M_PI) / 180.)

class LidarRepublisher : public rclcpp::Node
{
public:
  LidarRepublisher()
      : Node("lidar_republisher")
  {
    RCLCPP_INFO(this->get_logger(), "Starting lidar_republisher CPP node");

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Strip lidar data start angle in DEG";
    param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    param_desc.integer_range = {rcl_interfaces::msg::IntegerRange()
                                    .set__from_value(0)
                                    .set__to_value(360)
                                    .set__step(1)};
                                    

    this->declare_parameter("strip_angle_start", 0, param_desc);

    param_desc.description = "Strip lidar data stop angle in DEG";
    this->declare_parameter("strip_angle_stop", 0, param_desc);

    this->declare_parameter("input_scan", "scan_s1");
    this->declare_parameter("output_scan", "scan");    

    RCLCPP_INFO(this->get_logger(), "Init parameters: strip_angle_start: '%li', strip_angle_stop: '%li'",
                this->get_parameter("strip_angle_start").as_int(), this->get_parameter("strip_angle_stop").as_int());

    strip_angle_start_rad = DEG2RAD(this->get_parameter("strip_angle_start").as_int());
    strip_angle_stop_rad = DEG2RAD(this->get_parameter("strip_angle_stop").as_int());

    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        this->get_parameter("input_scan").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&LidarRepublisher::lidar_callback, this, _1));

    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(this->get_parameter("output_scan").as_string(), 10);

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    cb_handle1_ = param_subscriber_->add_parameter_callback("strip_angle_start",
                                                            std::bind(&LidarRepublisher::param_callback, this, _1));

    cb_handle2_ = param_subscriber_->add_parameter_callback("strip_angle_stop",
                                                            std::bind(&LidarRepublisher::param_callback, this, _1));
  }

private:
  void param_callback(const rclcpp::Parameter &p)
  {

    RCLCPP_DEBUG(
        this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
        p.get_name().c_str(),
        p.get_type_name().c_str(),
        p.as_int());

    if (p.get_name() == "strip_angle_start")
    {
      this->strip_angle_start_rad = DEG2RAD(p.as_int());
    }

    if (p.get_name() == "strip_angle_stop")
    {
      this->strip_angle_stop_rad = DEG2RAD(p.as_int());
    }
  }

  void lidar_callback(std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
  {
    RCLCPP_DEBUG(this->get_logger(), "Get lidar data time: '%i'", scan->header.stamp.sec);
    int count = scan->scan_time / scan->time_increment;

    if (strip_angle_start_rad > 0 and strip_angle_stop_rad > 0)
    {

      RCLCPP_DEBUG(this->get_logger(), "Have strip angles start: '%f', stop: '%f'", RAD2DEG(strip_angle_start_rad), RAD2DEG(strip_angle_stop_rad));

      for (int i = 0; i < count; i++)
      {
        auto angle = scan->angle_increment * i;
        if (angle >= strip_angle_start_rad && angle <= strip_angle_stop_rad)
        {
          scan->ranges[i] = INFINITY;
        }
      }
    }

    lidar_publisher_->publish(*scan);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle1_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle2_;

  float strip_angle_start_rad;
  float strip_angle_stop_rad;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRepublisher>());
  rclcpp::shutdown();
  return 0;
}
