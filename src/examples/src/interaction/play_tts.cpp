#include <aimdk_msgs/msg/tts_priority_level.hpp>
#include <aimdk_msgs/srv/play_tts.hpp>
#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

using PlayTTS = aimdk_msgs::srv::PlayTts;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("play_tts_client_min");

  const std::string service_name = "/aimdk_5Fmsgs/srv/PlayTts";
  auto client = node->create_client<PlayTTS>(service_name);

  RCLCPP_INFO(node->get_logger(), "✅ PlayTts client node created.");

  // Wait for the service to become available
  while (!client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "⏳ Service unavailable, waiting...");
  }
  RCLCPP_INFO(node->get_logger(),
              "🟢 Service available, ready to send request.");

  // Get text to speak
  std::string tts_text;
  if (argc > 1) {
    tts_text = argv[1];
  } else {
    std::cout << "Enter text to speak: ";
    if (!std::getline(std::cin, tts_text) || !rclcpp::ok()) {
      std::cout << std::endl;
      rclcpp::shutdown();
      return 1;
    }
    if (tts_text.empty()) {
      tts_text = "Hello, I am AgiBot X2.";
    }
  }

  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return 1;
  }

  auto req = std::make_shared<PlayTTS::Request>();
  req->tts_req.text = tts_text;
  req->tts_req.domain = "demo_client"; // Required: identifies the caller
  req->tts_req.trace_id =
      "demo"; // Optional: request identifier for the TTS request
  req->tts_req.is_interrupted =
      true; // Required: whether to interrupt same-priority playback
  req->tts_req.priority_weight = 0;
  req->tts_req.priority_level.value =
      aimdk_msgs::msg::TtsPriorityLevel::INTERACTION_L6; // L6 (6)

  RCLCPP_INFO(node->get_logger(), "📨 Sending request to play tts: text=%s",
              tts_text.c_str());

  std::shared_future<PlayTTS::Response::SharedPtr> future;
  rclcpp::FutureReturnCode rc = rclcpp::FutureReturnCode::TIMEOUT;
  for (int i = 0; i < 8; ++i) {
    req->header.header.stamp = node->now();
    future = client->async_send_request(req);
    rc = rclcpp::spin_until_future_complete(node, future,
                                            std::chrono::milliseconds(250));
    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      break;
    }
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      return 1;
    }
    // retry as remote peer is NOT handled well by ROS
    RCLCPP_INFO(node->get_logger(), "trying ... [%d]", i);
  }

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "❌ Service call not completed or timed out.");
    }
    rclcpp::shutdown();
    return 1;
  }

  const auto resp = future.get();
  if (resp->tts_resp.is_success) {
    RCLCPP_INFO(node->get_logger(), "✅ TTS sent successfully.");
  } else {
    RCLCPP_ERROR(node->get_logger(),
                 "❌ Failed to send TTS: error_code=%u, error_message=%s",
                 resp->tts_resp.error_code,
                 resp->tts_resp.error_message.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
