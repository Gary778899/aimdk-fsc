#include <chrono>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sstream>
#include <string>

class GnssPrinter : public rclcpp::Node {
public:
  GnssPrinter() : rclcpp::Node("gnss_printer") {
    constexpr char kNavSatTopic[] = "/aima/hal/sensor/gnss";

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliability(rclcpp::ReliabilityPolicy::Reliable)
                   .durability(rclcpp::DurabilityPolicy::TransientLocal);

    navsat_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        kNavSatTopic, qos,
        std::bind(&GnssPrinter::navsat_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribing NavSatFix: %s", kNavSatTopic);
  }

private:
  void navsat_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    auto now = std::chrono::steady_clock::now();
    double dt_ms =
        last_navsat_recv_.time_since_epoch().count() == 0
            ? 0.0
            : std::chrono::duration<double, std::milli>(now - last_navsat_recv_)
                  .count();
    last_navsat_recv_ = now;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "\n========== NavSatFix ==========\n";
    oss << "stamp=" << rclcpp::Time(msg->header.stamp).seconds() << "s\n";
    oss << "frame=" << msg->header.frame_id << "\n";
    oss << "recv_dt=" << std::setprecision(3) << dt_ms << "ms\n";
    oss << std::setprecision(8);
    oss << "latitude:  " << msg->latitude << " deg\n";
    oss << "longitude: " << msg->longitude << " deg\n";
    oss << std::setprecision(3);
    oss << "altitude:  " << msg->altitude << " m\n";

    const char *status_str = "UNKNOWN";
    switch (msg->status.status) {
    case sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX:
      status_str = "NO_FIX";
      break;
    case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
      status_str = "FIX";
      break;
    }
    oss << "status: " << status_str << "\n";
    oss << "service: " << msg->status.service << "\n";

    oss << std::setprecision(4);
    oss << "position_covariance: [";
    for (size_t i = 0; i < 9; ++i) {
      if (i > 0)
        oss << ", ";
      oss << msg->position_covariance[i];
    }
    oss << "]\n";
    oss << "position_covariance_type: "
        << static_cast<int>(msg->position_covariance_type);

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_sub_;
  std::chrono::steady_clock::time_point last_navsat_recv_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssPrinter>());
  rclcpp::shutdown();
  return 0;
}
