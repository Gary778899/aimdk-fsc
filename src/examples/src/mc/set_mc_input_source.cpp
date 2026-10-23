#include "aimdk_msgs/srv/set_mc_input_source.hpp"
#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_response.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/common_task_response.hpp"
#include "aimdk_msgs/msg/mc_input_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cinttypes>
#include <memory>
#include <signal.h>

namespace {
const char *action_name(int32_t value) {
  using A = aimdk_msgs::msg::McInputAction;
  switch (value) {
  case A::INPUTACTION_ADD:
    return "ADD";
  case A::INPUTACTION_MODIFY:
    return "MODIFY";
  case A::INPUTACTION_DELETE:
    return "DELETE";
  case A::INPUTACTION_ENABLE:
    return "ENABLE";
  case A::INPUTACTION_DISABLE:
    return "DISABLE";
  default:
    return "UNKNOWN";
  }
}
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

class McInputClient : public rclcpp::Node {
public:
  McInputClient() : Node("set_mc_input_source_client") {
    client_ = this->create_client<aimdk_msgs::srv::SetMcInputSource>(
        "/aimdk_5Fmsgs/srv/SetMcInputSource");

    RCLCPP_INFO(this->get_logger(), "✅ SetMcInputSource client node created.");

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  bool send_request() {
    try {
      auto request =
          std::make_shared<aimdk_msgs::srv::SetMcInputSource::Request>();

      // Set request data
      request->action.value =
          aimdk_msgs::msg::McInputAction::INPUTACTION_ADD; // ADD (1001)
      request->input_source.name = "node.set_mc_input_source";
      request->input_source.priority = 40;
      request->input_source.timeout = 1000;

      RCLCPP_INFO(this->get_logger(), "📨 Sending input source request: %s (%d)",
                  action_name(request->action.value), request->action.value);

      auto timeout = std::chrono::milliseconds(250);
      for (int i = 0; i < 8; i++) {
        // Set header timestamp
        request->request.header.stamp = this->now(); // use Node::now()
        auto future = client_->async_send_request(request);
        auto retcode = rclcpp::spin_until_future_complete(
            this->shared_from_this(), future, timeout);
        if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
          // retry as remote peer is NOT handled well by ROS
          RCLCPP_INFO(this->get_logger(), "trying ... [%d]", i);
          continue;
        }
        // future.done
        auto response = future.get();
        auto code = response->response.header.code;
        if (code == 0) {
          RCLCPP_INFO(this->get_logger(),
                      "✅ Input source set successfully. task_id=%" PRIu64,
                      response->response.task_id);
          return true;
        } else {
          RCLCPP_ERROR(
              this->get_logger(),
              "❌ Input source set failed. ret_code=%" PRId64
              ", task_id=%" PRIu64 " "
              "(duplicated ADD? or MODIFY/ENABLE/DISABLE for unknown source?)",
              code, response->response.task_id);
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
  rclcpp::Client<aimdk_msgs::srv::SetMcInputSource>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<McInputClient>();
    auto client = std::dynamic_pointer_cast<McInputClient>(g_node);

    if (client) {
      client->send_request();
    }

    g_node.reset();
    rclcpp::shutdown();

    return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program terminated with exception: %s", e.what());
    return 1;
  }
}
