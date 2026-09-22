#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Safety : public rclcpp::Node
{
public:
  Safety() : Node("safety_node")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10,
      std::bind(&Safety::odom_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&Safety::scan_callback, this, std::placeholders::_1));

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);
  }

private:
  static constexpr double TTC_THRESHOLD_SECONDS = 1.0;
  double speed_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    speed_ = msg->twist.twist.linear.x;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
  {
    double minimum_ttc = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      const double distance = scan_msg->ranges[i];

      if (!std::isfinite(distance) ||
          distance < scan_msg->range_min ||
          distance > scan_msg->range_max) {
        continue;
      }

      const double angle = scan_msg->angle_min +
        static_cast<double>(i) * scan_msg->angle_increment;
      const double closing_speed = std::max(speed_ * std::cos(angle), 0.0);

      if (closing_speed > 0.0) {
        const double ttc = distance / closing_speed;
        minimum_ttc = std::min(minimum_ttc, ttc);
      }
    }

    if (minimum_ttc < TTC_THRESHOLD_SECONDS) {
      ackermann_msgs::msg::AckermannDriveStamped brake_message;
      brake_message.drive.speed = 0.0;
      brake_message.drive.steering_angle = 0.0;
      drive_pub_->publish(brake_message);

      RCLCPP_WARN(
        this->get_logger(),
        "Emergency braking! TTC: %.2f seconds",
        minimum_ttc);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Safety>());
  rclcpp::shutdown();
  return 0;
}
