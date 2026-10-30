#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/common_task_response.hpp"
#include "aimdk_msgs/msg/mc_control_area.hpp"
#include "aimdk_msgs/msg/mc_input_action.hpp"
#include "aimdk_msgs/msg/mc_preset_motion.hpp"
#include "aimdk_msgs/msg/request_header.hpp"
#include "aimdk_msgs/srv/set_mc_input_source.hpp"
#include "aimdk_msgs/srv/set_mc_preset_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cinttypes>
#include <iostream>
#include <memory>
#include <signal.h>

namespace {
constexpr const char *kInputSourceName = "node.preset_motion";
constexpr int32_t kInputSourcePriority = 40;
constexpr int32_t kInputSourceTimeout = 1000;
} // namespace

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class PresetMotionClient : public rclcpp::Node {
public:
  PresetMotionClient() : Node("preset_motion_client") {
    client_ = this->create_client<aimdk_msgs::srv::SetMcPresetMotion>(
        "/aimdk_5Fmsgs/srv/SetMcPresetMotion");
    input_source_client_ =
        this->create_client<aimdk_msgs::srv::SetMcInputSource>(
            "/aimdk_5Fmsgs/srv/SetMcInputSource");

    RCLCPP_INFO(this->get_logger(), "✅ SetMcPresetMotion client node created.");

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  bool register_input_source() {
    if (!input_source_client_->wait_for_service(std::chrono::seconds(8))) {
      RCLCPP_ERROR(this->get_logger(),
                   "❌ SetMcInputSource service unavailable.");
      return false;
    }

    // Try ADD first; if it fails (likely already registered), fall back to
    // ENABLE
    if (call_input_source( // ADD (1001)
            aimdk_msgs::msg::McInputAction::INPUTACTION_ADD)) {
      RCLCPP_INFO(this->get_logger(), "✅ Input source \"%s\" registered (ADD).",
                  kInputSourceName);
      return true;
    }
    RCLCPP_INFO(this->get_logger(),
                "ADD failed (likely already registered), trying ENABLE...");
    if (call_input_source( // ENABLE (2001)
            aimdk_msgs::msg::McInputAction::INPUTACTION_ENABLE)) {
      RCLCPP_INFO(this->get_logger(), "✅ Input source \"%s\" enabled.",
                  kInputSourceName);
      return true;
    }
    RCLCPP_ERROR(this->get_logger(),
                 "❌ Failed to register input source via ADD or ENABLE.");
    return false;
  }

  bool send_request(int area_id, int motion_id) {
    try {
      auto request =
          std::make_shared<aimdk_msgs::srv::SetMcPresetMotion::Request>();
      request->header = aimdk_msgs::msg::RequestHeader();

      request->input_source.name = kInputSourceName;
      request->input_source.priority = kInputSourcePriority;
      request->input_source.timeout = kInputSourceTimeout;

      aimdk_msgs::msg::McPresetMotion motion;
      aimdk_msgs::msg::McControlArea area;

      motion.value = motion_id; // Preset motion ID
      area.value = area_id;     // Control area ID
      request->motion = motion;
      request->area = area;
      request->interrupt = false; // Not interrupt current motion

      RCLCPP_INFO(this->get_logger(),
                  "📨 Sending request to set preset motion: motion=%d, area=%d",
                  motion_id, area_id);

      const std::chrono::milliseconds timeout(250);
      for (int i = 0; i < 8; i++) {
        request->header.stamp = this->now();
        auto future = client_->async_send_request(request);
        auto retcode = rclcpp::spin_until_future_complete(shared_from_this(),
                                                          future, timeout);
        if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
          // retry as remote peer is NOT handled well by ROS
          RCLCPP_INFO(this->get_logger(), "trying ... [%d]", i);
          continue;
        }
        // future.done
        auto response = future.get();
        if (response->response.header.code == 0) {
          RCLCPP_INFO(this->get_logger(),
                      "✅ Preset motion set successfully: %" PRIu64,
                      response->response.task_id);
          return true;
        } else if (response->response.state.value ==
                   aimdk_msgs::msg::CommonState::RUNNING) { // RUNNING (400)
          RCLCPP_INFO(this->get_logger(), "⏳ Preset motion executing: %" PRIu64,
                      response->response.task_id);
          return true;
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "❌ Failed to set preset motion: %" PRIu64,
                      response->response.task_id);
          return false;
        }
      }
      RCLCPP_ERROR(this->get_logger(), "❌ Service call failed or timed out.");
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  bool call_input_source(int32_t action_value) {
    auto req = std::make_shared<aimdk_msgs::srv::SetMcInputSource::Request>();
    req->action.value = action_value;
    req->input_source.name = kInputSourceName;
    req->input_source.priority = kInputSourcePriority;
    req->input_source.timeout = kInputSourceTimeout;

    const std::chrono::milliseconds timeout(250);
    for (int i = 0; i < 8; i++) {
      req->request.header.stamp = this->now();
      auto future = input_source_client_->async_send_request(req);
      auto retcode = rclcpp::spin_until_future_complete(shared_from_this(),
                                                        future, timeout);
      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        // retry as remote peer is NOT handled well by ROS
        RCLCPP_INFO(
            this->get_logger(), "trying input source action %s (%d)... [%d]",
            action_value == aimdk_msgs::msg::McInputAction::INPUTACTION_ADD
                ? "ADD"
                : "ENABLE",
            action_value, i);
        continue;
      }
      auto response = future.get();
      return response->response.header.code == 0;
    }
    return false;
  }

  rclcpp::Client<aimdk_msgs::srv::SetMcPresetMotion>::SharedPtr client_;
  rclcpp::Client<aimdk_msgs::srv::SetMcInputSource>::SharedPtr
      input_source_client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<PresetMotionClient>();
    // Cast g_node (std::shared_ptr<rclcpp::Node>) to a derived
    // PresetMotionClient pointer (std::shared_ptr<PresetMotionClient>)
    auto client = std::dynamic_pointer_cast<PresetMotionClient>(g_node);

    int area = 1;
    int motion = 1003;
    std::cout << "Enter arm area ID (1-left, 2-right): ";
    std::cin >> area;
    std::cout
        << "Enter preset motion ID (1001-raise, 1002-wave, 1003-handshake, "
           "1004-airkiss): ";
    std::cin >> motion;
    if (client) {
      if (!client->register_input_source()) {
        RCLCPP_ERROR(g_node->get_logger(),
                     "❌ Input source registration failed, exiting");
      } else {
        client->send_request(area, motion);
      }
    }

    // Clean up resources
    g_node.reset();
    rclcpp::shutdown();

    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
