#include <aimdk_msgs/msg/common_state.hpp>
#include <aimdk_msgs/srv/play_audio_file.hpp>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

using PlayAudioFile = aimdk_msgs::srv::PlayAudioFile;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("play_audio_client_min");

  // 1) Service name
  const std::string service_name = "/aimdk_5Fmsgs/srv/PlayAudioFile";
  auto client = node->create_client<PlayAudioFile>(service_name);

  RCLCPP_INFO(node->get_logger(), "✅ PlayAudio client node created.");

  // 2) Input file path (prompt user if not provided as argument)
  std::string default_file = "/agibot/data/var/hal_audio/file/bingo.wav";
  std::string file_name;

  if (argc > 1) {
    file_name = argv[1];
  } else {
    std::cout << "Enter audio file path to play (default: " << default_file
              << "): ";
    if (!std::getline(std::cin, file_name) || !rclcpp::ok()) {
      std::cout << std::endl;
      rclcpp::shutdown();
      return 1;
    }
    // Trim leading/trailing whitespace (parity with Python's .strip())
    auto not_space = [](int c) { return !std::isspace(c); };
    file_name.erase(
        file_name.begin(),
        std::find_if(file_name.begin(), file_name.end(), not_space));
    file_name.erase(
        std::find_if(file_name.rbegin(), file_name.rend(), not_space).base(),
        file_name.end());
    if (file_name.empty()) {
      file_name = default_file;
    }
  }

  // 3) Build the request
  // Check if Ctrl+C was pressed during user input
  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return 1;
  }

  auto req = std::make_shared<PlayAudioFile::Request>();
  req->file.pkg_name = "demo_client";
  req->file.file_name = std::filesystem::path(file_name).filename().string();
  req->file.file_path =
      std::filesystem::path(file_name).parent_path().string() + "/";
  req->file.priority = 8;
  req->file.priority_weight = 0;

  // 4) Wait for service
  while (!client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "⏳ Service unavailable, waiting...");
  }
  RCLCPP_INFO(node->get_logger(),
              "🟢 Service available, ready to send request.");

  RCLCPP_INFO(node->get_logger(), "📨 Sending request to play audio: %s",
              file_name.c_str());

  rclcpp::FutureReturnCode rc = rclcpp::FutureReturnCode::TIMEOUT;
  std::shared_future<PlayAudioFile::Response::SharedPtr> future;
  for (int i = 0; i < 5; ++i) {
    req->request.header.stamp = node->now();
    future = client->async_send_request(req);
    rc = rclcpp::spin_until_future_complete(node, future,
                                            std::chrono::seconds(1));
    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      break;
    }
    if (rc == rclcpp::FutureReturnCode::INTERRUPTED) {
      // Check if Ctrl+C was pressed during spin_until_future_complete
      if (rclcpp::ok()) {
        RCLCPP_WARN(node->get_logger(), "Interrupted while waiting");
      }
      rclcpp::shutdown();
      return 1;
    }
    // retry as remote peer is NOT handled well by ROS
    RCLCPP_INFO(node->get_logger(), "trying ... [%d]", i);
  }

  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    // Check if Ctrl+C was pressed, avoid using node after shutdown
    if (rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "❌ Service call not completed or timed out.");
    }
    rclcpp::shutdown();
    return 1;
  }

  // 5) Handle response (success is in reponse.status)
  try {
    const auto resp = future.get();
    bool success = resp->reponse.status.value ==
                   aimdk_msgs::msg::CommonState::SUCCESS; // SUCCESS (1)

    if (success) {
      RCLCPP_INFO(node->get_logger(),
                  "✅ Request to play audio file recorded successfully.");
    } else {
      // FAILURE(2): focus-apply failed; INVALID(5): invalid params;
      // other: see CommonState for details.
      const char *reason = "unknown";
      switch (resp->reponse.status.value) {
      case aimdk_msgs::msg::CommonState::FAILURE: // FAILURE (2)
        reason = "focus apply failed";
        break;
      case aimdk_msgs::msg::CommonState::INVALID: // INVALID (5)
        reason = "invalid params";
        break;
      default:
        break;
      }
      RCLCPP_ERROR(node->get_logger(),
                   "❌ Failed to record play-audio request (status=%d, %s): %s",
                   resp->reponse.status.value, reason,
                   resp->reponse.message.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Call exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
