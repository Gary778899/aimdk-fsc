#include "aimdk_msgs/srv/play_video.hpp"
#include "aimdk_msgs/msg/common_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <memory>
#include <signal.h>
#include <string>

// Global variable used for signal handling
std::shared_ptr<rclcpp::Node> g_node = nullptr;

// Signal handler function
void signal_handler(int signal) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...",
                signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class PlayVideoClient : public rclcpp::Node {
public:
  PlayVideoClient() : Node("play_video_client") {
    client_ = this->create_client<aimdk_msgs::srv::PlayVideo>(
        "/aimdk_5Fmsgs/srv/PlayVideo");
    RCLCPP_INFO(this->get_logger(), "✅ PlayVideo client node started.");

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  bool send_request(const std::string &video_path, uint8_t mode,
                    int32_t priority) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::PlayVideo::Request>();

      request->video_path = video_path;
      request->mode = mode;
      request->priority = priority;

      RCLCPP_INFO(this->get_logger(),
                  "📨 Sending request to play video: mode=%hhu video=%s", mode,
                  video_path.c_str());

      const std::chrono::milliseconds timeout(250);
      for (int i = 0; i < 8; i++) {
        request->header.header.stamp = this->now();
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
        if (response->success) {
          RCLCPP_INFO(this->get_logger(),
                      "✅ Request to play video recorded successfully: %s",
                      response->message.c_str());
          return true;
        } else {
          RCLCPP_ERROR(this->get_logger(),
                       "❌ Failed to record play-video request: %s",
                       response->message.c_str());
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
  rclcpp::Client<aimdk_msgs::srv::PlayVideo>::SharedPtr client_;
};

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);

    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::string default_video_path =
        "/agibot/software/face_ui/bin/app/aarch64-generic/assets/mp4/"
        "emoji_x2_sleepy.mp4";
    std::string video_path;
    std::cout << "Enter video path (default: " << default_video_path << "): ";
    std::getline(std::cin, video_path);
    // Trim leading/trailing whitespace (parity with Python's .strip())
    auto not_space = [](int c) { return !std::isspace(c); };
    video_path.erase(
        video_path.begin(),
        std::find_if(video_path.begin(), video_path.end(), not_space));
    video_path.erase(
        std::find_if(video_path.rbegin(), video_path.rend(), not_space).base(),
        video_path.end());
    if (video_path.empty()) {
      video_path = default_video_path;
    }
    int32_t priority = 5;
    int mode = 2; // Loop playback
    std::cout << "Enter video play mode (1: once, 2: loop): ";
    std::cin >> mode;
    if (mode < 1 || mode > 2) {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Invalid play mode: %d", mode);
      rclcpp::shutdown();
      return 1;
    }

    g_node = std::make_shared<PlayVideoClient>();
    auto client = std::dynamic_pointer_cast<PlayVideoClient>(g_node);

    if (client) {
      client->send_request(video_path, mode, priority);
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
