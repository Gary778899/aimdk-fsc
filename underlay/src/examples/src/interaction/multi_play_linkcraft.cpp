//
// Created by agiuser on 2026/3/3.
//

#include <aimdk_msgs/msg/slave_device.hpp>
#include <aimdk_msgs/srv/execute_action_resource.hpp>
#include <aimdk_msgs/srv/get_robot_resources.hpp>
#include <aimdk_msgs/srv/group_control_time_sync.hpp>
#include <cinttypes>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class MultiRobotResourceClient : public rclcpp::Node {
public:
  MultiRobotResourceClient() : Node("robot_resource_client") {
    get_resource_client_ =
        this->create_client<aimdk_msgs::srv::GetRobotResources>(
            "/aimdk_5Fmsgs/srv/GetRobotResources");
    play_resource_client_ =
        this->create_client<aimdk_msgs::srv::ExecuteActionResource>(
            "/aimdk_5Fmsgs/srv/ExecuteActionResource");
    time_sync_client_ =
        this->create_client<aimdk_msgs::srv::GroupControlTimeSync>(
            "/aimdk_5Fmsgs/srv/GroupControlTimeSync");
  }

  bool get_robot_resources() {
    while (!get_resource_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service not available, waiting again...");
    }

    if (!rclcpp::ok()) {
      return false;
    }

    auto request =
        std::make_shared<aimdk_msgs::srv::GetRobotResources::Request>();

    auto future_result = get_resource_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call GetRobotResources");
      }
      return false;
    }

    auto response = future_result.get();
    resources_ = response->robot_resources;
    RCLCPP_INFO(this->get_logger(), "Found %zu resources", resources_.size());
    return true;
  }

  void print_resource_menu() {
    if (resources_.empty()) {
      std::cout << "(no resources)\n";
      return;
    }

    std::cout << "\n=== Available Robot Resources ===\n";
    for (size_t i = 0; i < resources_.size(); ++i) {
      const auto &ver = resources_[i].current_version;
      std::cout << "[" << i << "] "
                << (ver.name.empty() ? "<no name>" : ver.name) << "\n";
      std::cout << "     key:     " << resources_[i].resource_key << "\n";
      std::cout << "     version: " << ver.version << "\n";
    }
    std::cout << "=================================\n\n";
  }

  int pick_resource() {
    if (resources_.empty()) {
      return -1;
    }

    std::string line;
    while (true) {
      std::cout << "Select resource index [0-" << resources_.size() - 1
                << "] (q to quit): ";
      if (!std::getline(std::cin, line)) {
        return -1;
      }
      if (line == "q" || line == "quit" || line == "exit") {
        return -1;
      }
      try {
        int idx = std::stoi(line);
        if (idx >= 0 && idx < static_cast<int>(resources_.size())) {
          return idx;
        }
        std::cout << "Index out of range (0-" << resources_.size() - 1
                  << ").\n";
      } catch (...) {
        std::cout << "Please enter a number.\n";
      }
    }
  }

  void play_resource(int idx) {
    while (!play_resource_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }

    if (!rclcpp::ok()) {
      return;
    }

    const auto &resource = resources_[idx];
    auto request =
        std::make_shared<aimdk_msgs::srv::ExecuteActionResource::Request>();

    request->resource_key = resource.resource_key;
    request->resource_version = resource.current_version.version;

    for (const auto &ip : slave_ips_) {
      aimdk_msgs::msg::SlaveDevice slave;
      slave.ip = ip;
      request->slaves.push_back(slave);
    }

    if (request->resource_key.find("onnx") != std::string::npos) {
      request->meta =
          R"({"control_type": "GROUP_CONTROL","resource_type": "BODY_MONTION"})";
    } else {
      request->meta =
          R"({"control_type": "GROUP_CONTROL","resource_type": "ARM_MONTION"})";
    }

    RCLCPP_INFO(this->get_logger(), "Sending request:");
    RCLCPP_INFO(this->get_logger(), "  resource_key: %s",
                request->resource_key.c_str());
    RCLCPP_INFO(this->get_logger(), "  resource_version: %s",
                request->resource_version.c_str());
    RCLCPP_INFO(this->get_logger(), "  meta: %s", request->meta.c_str());
    for (size_t i = 0; i < request->slaves.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  slaves[%zu] ip: %s", i,
                  request->slaves[i].ip.c_str());
    }

    auto future = play_resource_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      handle_response(future.get());
    } else {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to call ExecuteActionResource");
      }
    }
  }

  bool sync_time(const std::vector<std::string> &ips) {
    slave_ips_ = ips;

    while (!time_sync_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
                  "TimeSync service not available, waiting again...");
    }

    if (!rclcpp::ok()) {
      return false;
    }

    auto request =
        std::make_shared<aimdk_msgs::srv::GroupControlTimeSync::Request>();

    for (const auto &ip : ips) {
      aimdk_msgs::msg::SlaveDevice slave;
      slave.ip = ip;
      request->slaves.push_back(slave);
    }

    RCLCPP_INFO(this->get_logger(), "Sending GroupControlTimeSync request:");
    for (const auto &slave : request->slaves) {
      RCLCPP_INFO(this->get_logger(), "  slave ip: %s", slave.ip.c_str());
    }

    auto future = time_sync_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call GroupControlTimeSync");
      }
      return false;
    }

    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "GroupControlTimeSync response:");
    RCLCPP_INFO(this->get_logger(), "  error_code: %d", response->error_code);
    RCLCPP_INFO(this->get_logger(), "  error_msg: %s",
                response->error_msg.c_str());
    return response->error_code == 0;
  }

private:
  rclcpp::Client<aimdk_msgs::srv::GetRobotResources>::SharedPtr
      get_resource_client_;
  rclcpp::Client<aimdk_msgs::srv::ExecuteActionResource>::SharedPtr
      play_resource_client_;
  rclcpp::Client<aimdk_msgs::srv::GroupControlTimeSync>::SharedPtr
      time_sync_client_;
  std::vector<aimdk_msgs::msg::RobotResource> resources_;
  std::vector<std::string> slave_ips_;

  void handle_response(
      const aimdk_msgs::srv::ExecuteActionResource::Response::SharedPtr
          response) {
    RCLCPP_INFO(this->get_logger(), "Service response received:");
    RCLCPP_INFO(this->get_logger(), "Response Header:");
    RCLCPP_INFO(this->get_logger(), "  stamp: %d.%09u",
                response->header.header.stamp.sec,
                response->header.header.stamp.nanosec);
    RCLCPP_INFO(this->get_logger(), "  code: %" PRId64,
                response->header.header.code);
    RCLCPP_INFO(this->get_logger(), "Status:");
    RCLCPP_INFO(this->get_logger(), "  value: %d",
                response->header.status.value);
    RCLCPP_INFO(this->get_logger(), "  message: %s",
                response->header.message.c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<MultiRobotResourceClient>();

  if (!client_node->get_robot_resources()) {
    rclcpp::shutdown();
    return 1;
  }

  // [USER_CONFIG] Replace the IPs below with your slave robot IPs.
  std::vector<std::string> slave_ips = {"172.22.133.178", "172.22.135.45"};
  if (!client_node->sync_time(slave_ips)) {
    RCLCPP_WARN(client_node->get_logger(),
                "Time sync failed or returned error.");
  }

  client_node->print_resource_menu();
  int chosen = client_node->pick_resource();
  if (chosen < 0) {
    RCLCPP_INFO(client_node->get_logger(), "No resource selected, exiting.");
    rclcpp::shutdown();
    return 0;
  }

  client_node->play_resource(chosen);
  rclcpp::shutdown();
  return 0;
}
