#include <chrono>
#include <cinttypes>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <aimdk_msgs/msg/message_header.hpp>
#include <aimdk_msgs/msg/pnc_task_request.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

// task_request enum
constexpr uint8_t TASK_REQUEST_START = 1;
constexpr uint8_t TASK_REQUEST_STOP = 2;
constexpr uint8_t TASK_REQUEST_PAUSE = 3;
constexpr uint8_t TASK_REQUEST_RESUME = 4;

class NavigationNode : public rclcpp::Node {
public:
  NavigationNode() : Node("navigation_example") {
    publisher_ = this->create_publisher<aimdk_msgs::msg::PncTaskRequest>(
        "/aima/te/pnc_task_request", 10);

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
    task_id_ = static_cast<uint64_t>(now_ms);
    map_id_ = 0;

    RCLCPP_INFO(this->get_logger(),
                "Navigation example node started, task_id=%" PRIu64, task_id_);
  }

  void send_start(uint64_t map_id, double x, double y, double z, double ox,
                  double oy, double oz, double ow, double radius,
                  uint8_t pnc_mode, double max_speed) {
    map_id_ = map_id;
    auto msg = aimdk_msgs::msg::PncTaskRequest();
    msg.header = make_header();
    msg.task_type = 2;
    msg.task_request = TASK_REQUEST_START;
    msg.pnc_mode = pnc_mode;
    msg.max_forward_speed = max_speed;
    msg.task_id = task_id_;
    msg.map_id = map_id;
    msg.target_pose_radius = radius;
    std::memset(msg.reserve_info.data(), 0, msg.reserve_info.size());

    auto &target = msg.target_pose;
    target.header.stamp = this->get_clock()->now();
    target.header.frame_id = "map";
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation.x = ox;
    target.pose.orientation.y = oy;
    target.pose.orientation.z = oz;
    target.pose.orientation.w = ow;

    RCLCPP_INFO(this->get_logger(),
                "[Start] task_id=%" PRIu64 ", map_id=%" PRIu64 ", "
                "target=(%.2f, %.2f, %.2f), radius=%.2f",
                task_id_, map_id, x, y, z, radius);
    start_publishing(msg);
  }

  void send_pause() {
    auto msg = make_simple_request(TASK_REQUEST_PAUSE, 0);
    RCLCPP_INFO(this->get_logger(), "[Pause] task_id=%" PRIu64, task_id_);
    start_publishing(msg);
  }

  void send_resume() {
    auto msg = make_simple_request(TASK_REQUEST_RESUME, map_id_);
    RCLCPP_INFO(this->get_logger(),
                "[Resume] task_id=%" PRIu64 ", map_id=%" PRIu64, task_id_,
                map_id_);
    start_publishing(msg);
  }

  void send_stop() {
    auto msg = make_simple_request(TASK_REQUEST_STOP, map_id_);
    RCLCPP_INFO(this->get_logger(),
                "[Stop] task_id=%" PRIu64 ", map_id=%" PRIu64, task_id_,
                map_id_);
    start_publishing(msg);
  }

  void stop_publishing() {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
      RCLCPP_INFO(this->get_logger(), "Stopped continuous publishing");
    }
  }

  uint64_t task_id() const { return task_id_; }

private:
  void start_publishing(const aimdk_msgs::msg::PncTaskRequest &msg) {
    current_msg_ = msg;
    if (timer_) {
      timer_->cancel();
    }
    publisher_->publish(current_msg_);
    timer_ = this->create_wall_timer(1s, [this]() {
      current_msg_.header.stamp = this->get_clock()->now();
      current_msg_.header.meas_stamp = this->get_clock()->now();
      current_msg_.target_pose.header.stamp = this->get_clock()->now();
      publisher_->publish(current_msg_);
    });
    RCLCPP_INFO(this->get_logger(), "Continuous publishing at 1Hz");
  }

  aimdk_msgs::msg::MessageHeader make_header() {
    auto header = aimdk_msgs::msg::MessageHeader();
    auto now = this->get_clock()->now();
    header.stamp = now;
    header.meas_stamp = now;
    header.frame_id = "map";
    header.sequence = 0;
    return header;
  }

  aimdk_msgs::msg::PncTaskRequest make_simple_request(uint8_t task_request,
                                                      uint64_t map_id) {
    auto msg = aimdk_msgs::msg::PncTaskRequest();
    msg.header = make_header();
    msg.task_type = 2;
    msg.task_request = task_request;
    msg.pnc_mode = 0;
    msg.max_forward_speed = 0.5;
    msg.task_id = task_id_;
    msg.map_id = map_id;
    msg.target_pose_radius = 0.0;
    std::memset(msg.reserve_info.data(), 0, msg.reserve_info.size());
    msg.target_pose.pose.orientation.w = 1.0;
    return msg;
  }

  rclcpp::Publisher<aimdk_msgs::msg::PncTaskRequest>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  aimdk_msgs::msg::PncTaskRequest current_msg_;
  uint64_t task_id_;
  uint64_t map_id_;
};

template <typename T> T input_value(const std::string &prompt, T default_val) {
  std::cout << prompt << " [" << default_val << "]: ";
  std::string line;
  std::getline(std::cin, line);
  if (line.empty()) {
    return default_val;
  }
  if constexpr (std::is_same_v<T, uint64_t>) {
    return std::stoull(line);
  } else if constexpr (std::is_same_v<T, uint8_t>) {
    return static_cast<uint8_t>(std::stoi(line));
  } else if constexpr (std::is_integral_v<T>) {
    return static_cast<T>(std::stoll(line));
  } else {
    return static_cast<T>(std::stod(line));
  }
}

void print_menu(uint64_t task_id) {
  std::cout << "\n========================================\n"
            << "  Navigation Control  (task_id=" << task_id << ")\n"
            << "========================================\n"
            << "  1. Start navigation\n"
            << "  2. Pause navigation\n"
            << "  3. Resume navigation\n"
            << "  4. Stop navigation\n"
            << "  q. Quit\n"
            << "========================================\n";
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationNode>();

  std::thread spin_thread([node]() { rclcpp::spin(node); });

  RCLCPP_INFO(node->get_logger(),
              "Waiting for node communication ready (5s)...");
  std::this_thread::sleep_for(5s);

  std::string choice;
  while (rclcpp::ok()) {
    print_menu(node->task_id());
    std::cout << "Select an option: ";
    std::getline(std::cin, choice);

    if (choice == "1") {
      std::cout << "\n--- Start Navigation Parameters ---\n";
      auto map_id = input_value<uint64_t>("Map ID (map_id)", 1773113429735ULL);
      auto x = input_value<double>("Target x", 1.0);
      auto y = input_value<double>("Target y", 2.0);
      auto z = input_value<double>("Target z", 0.0);
      auto ox = input_value<double>("Orientation x", 0.0);
      auto oy = input_value<double>("Orientation y", 0.0);
      auto oz = input_value<double>("Orientation z", 0.0);
      auto ow = input_value<double>("Orientation w", 1.0);
      auto radius = input_value<double>("Target pose radius", 0.5);
      auto pnc_mode = input_value<uint8_t>("pnc_mode", 0);
      auto max_speed = input_value<double>("Max forward speed", 0.5);
      node->send_start(map_id, x, y, z, ox, oy, oz, ow, radius, pnc_mode,
                       max_speed);
    } else if (choice == "2") {
      node->send_pause();
    } else if (choice == "3") {
      node->send_resume();
    } else if (choice == "4") {
      node->send_stop();
    } else if (choice == "q" || choice == "Q") {
      node->stop_publishing();
      std::cout << "Exiting navigation control.\n";
      break;
    } else {
      std::cout << "Invalid input, please try again.\n";
    }
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
