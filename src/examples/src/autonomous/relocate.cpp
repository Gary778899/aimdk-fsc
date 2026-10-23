#include <chrono>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace std::chrono_literals;

class RelocalizationNode : public rclcpp::Node {
public:
  RelocalizationNode(uint64_t map_id, double init_x, double init_y)
      : Node("relocalization_node"), map_id_(map_id), init_x_(init_x),
        init_y_(init_y) {
    // /integrated_command subscriber requires TRANSIENT_LOCAL durability
    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    command_qos.transient_local();

    integrated_command_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/integrated_command", command_qos);
    relocalization_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
        "/relocalization_pose", 10);

    // Create subscriber with BEST_EFFORT QoS
    auto lidar_loc_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    lidar_loc_qos.best_effort();

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/slam/lidar_odom", lidar_loc_qos,
        std::bind(&RelocalizationNode::odometryCallback, this,
                  std::placeholders::_1));

    // Wait for subscribers before publishing
    publish_timer_ = this->create_wall_timer(200ms, [this]() {
      // Wait until both publishers have at least one subscriber
      if (integrated_command_pub_->get_subscription_count() == 0 ||
          relocalization_pose_pub_->get_subscription_count() == 0) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for subscribers... (command: %zu, pose: %zu)",
            integrated_command_pub_->get_subscription_count(),
            relocalization_pose_pub_->get_subscription_count());
        return;
      }
      publish_timer_->cancel();

      // Publish integrated_command
      auto integrated_command_msg = std_msgs::msg::String();
      integrated_command_msg.data =
          "start_relocalization:" + std::to_string(map_id_);
      integrated_command_pub_->publish(integrated_command_msg);
      RCLCPP_INFO(this->get_logger(), "Published integrated_command");

      // Schedule relocalization_pose publication after delay
      this->relocalization_pose_timer_ = this->create_wall_timer(1s, [this]() {
        this->relocalization_pose_timer_->cancel();

        auto relocalization_pose_msg = geometry_msgs::msg::Pose();
        relocalization_pose_msg.position.x = init_x_;
        relocalization_pose_msg.position.y = init_y_;
        relocalization_pose_msg.position.z = 0.0;
        relocalization_pose_msg.orientation.x = 0.0;
        relocalization_pose_msg.orientation.y = 0.0;
        relocalization_pose_msg.orientation.z = 0.0;
        relocalization_pose_msg.orientation.w = 1.0;

        relocalization_pose_pub_->publish(relocalization_pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published relocalization_pose");

        // Start timeout timer (30 seconds)
        this->timeout_timer_ = this->create_wall_timer(30s, [this]() {
          if (!success_received_) {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout reached - Relocalization failed!");
            rclcpp::shutdown();
          }
        });

        RCLCPP_INFO(this->get_logger(),
                    "Waiting for robot pose data (timeout: 30s)...");
      });
    });
  }

private:
  // get robot pose here
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!success_received_) {
      success_received_ = true;
      if (timeout_timer_) {
        timeout_timer_->cancel();
      }
      RCLCPP_INFO(this->get_logger(),
                  "Received odometry data - Relocalization successful!");
      rclcpp::shutdown();
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr integrated_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr
      relocalization_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr relocalization_pose_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  uint64_t map_id_;
  double init_x_;
  double init_y_;
  bool success_received_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Inputs needed for relocalization. To find them, see:
  //   map_id        -> sqlite3 /agibot/data/var/MapManagerModule/map.db
  //                    "SELECT * FROM map;"
  //   init_x/init_y -> origin (u, v) in
  //                    /agibot/data/var/MapManagerModule/<map_id>/grid_map_info.txt
  //                    (line 2; pixel coordinates)
  uint64_t map_id = 0;
  double init_x = 0.0;
  double init_y = 0.0;
  try {
    std::string line;
    std::cout << "Enter map_id: ";
    std::getline(std::cin, line);
    map_id = std::stoull(line);
    std::cout << "Enter initial position.x (pixel u): ";
    std::getline(std::cin, line);
    init_x = std::stod(line);
    std::cout << "Enter initial position.y (pixel v): ";
    std::getline(std::cin, line);
    init_y = std::stod(line);
  } catch (const std::exception &e) {
    std::cerr << "Invalid input, aborted." << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<RelocalizationNode>(map_id, init_x, init_y);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
