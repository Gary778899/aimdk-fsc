#include <aimdk_msgs/msg/common_request.hpp>
#include <aimdk_msgs/srv/set_pmu_led.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <string>

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

class PlayLightsClient : public rclcpp::Node {
public:
  PlayLightsClient() : Node("play_lights_client") {
    client_ = this->create_client<aimdk_msgs::srv::SetPmuLed>(
        "/aimdk_5Fmsgs/srv/SetPmuLed");
    RCLCPP_INFO(this->get_logger(), "✅ PlayLights client node started.");

    // Wait for the service to become available
    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  bool send_request(uint8_t led_mode, uint8_t r, uint8_t g, uint8_t b,
                    int32_t priority = 6, bool reset_priority = false) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetPmuLed::Request>();

      request->led_strip_mode = led_mode;
      request->r = r;
      request->g = g;
      request->b = b;
      request->priority = priority;
      request->reset_priority = reset_priority;
      request->trace_id = "play_lights";

      RCLCPP_INFO(this->get_logger(),
                  "📨 Sending request to control led strip: mode=%hhu, "
                  "RGB=(%hhu, %hhu, %hhu), priority=%d, reset_priority=%s",
                  led_mode, r, g, b, priority,
                  reset_priority ? "true" : "false");

      // LED strip is slow to response (up to ~5s)
      const std::chrono::milliseconds timeout(5000);
      for (int i = 0; i < 4; i++) {
        request->request.header.stamp = this->now();
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
        if (response->status_code == 0) {
          RCLCPP_INFO(this->get_logger(),
                      "✅ LED strip command sent successfully.");
          return true;
        } else {
          // 0x1024: priority insufficient (priority < current threshold);
          // use a higher priority or set reset_priority=true to reset the
          // threshold to 0.
          RCLCPP_ERROR(this->get_logger(),
                       "❌ LED strip command failed with status: 0x%X (%s)",
                       response->status_code,
                       response->status_code == 0x1024 ? "priority insufficient"
                                                       : "failure");
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
  rclcpp::Client<aimdk_msgs::srv::SetPmuLed>::SharedPtr client_;
};

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_node = std::make_shared<PlayLightsClient>();
    auto client_node = std::dynamic_pointer_cast<PlayLightsClient>(g_node);

    int led_mode = 0;          // LED Strip Mode
    int r = 255, g = 0, b = 0; // RGB values
    int priority = 6;          // Priority
    bool reset_priority = false;

    // Parse command-line arguments: mode R G B priority reset_priority
    // e.g. ros2 run examples play_lights 1 255 0 0 6 false
    if (argc > 6) {
      led_mode = std::stoi(argv[1]);
      if (led_mode < 0 || led_mode > 3) {
        throw std::runtime_error("invalid mode");
      }
      r = std::stoi(argv[2]);
      if (r < 0 || r > 255) {
        throw std::runtime_error("invalid R value");
      }
      g = std::stoi(argv[3]);
      if (g < 0 || g > 255) {
        throw std::runtime_error("invalid G value");
      }
      b = std::stoi(argv[4]);
      if (b < 0 || b > 255) {
        throw std::runtime_error("invalid B value");
      }
      priority = std::stoi(argv[5]);
      if (priority < 0) {
        throw std::runtime_error("invalid priority value");
      }
      std::string reset_str = argv[6];
      std::transform(reset_str.begin(), reset_str.end(), reset_str.begin(),
                     ::tolower);
      reset_priority = (reset_str == "true" || reset_str == "1" ||
                        reset_str == "yes" || reset_str == "y");
    } else {
      std::cout << "=== LED strip control example ===" << std::endl;
      std::cout << "Select LED strip mode:" << std::endl;
      std::cout << "0 - Steady on" << std::endl;
      std::cout << "1 - Breathing (4s cycle, sine brightness)" << std::endl;
      std::cout << "2 - Blinking (1s cycle, 0.5s on, 0.5s off)" << std::endl;
      std::cout << "3 - Flowing (2s cycle, light up from left to right)"
                << std::endl;
      std::cout << "Enter mode (0-3): ";
      std::cin >> led_mode;
      if (led_mode < 0 || led_mode > 3) {
        throw std::runtime_error("invalid mode");
      }

      std::cout << "\nSet RGB color values (0-255):" << std::endl;
      std::cout << "Red (R): ";
      std::cin >> r;
      if (r < 0 || r > 255) {
        throw std::runtime_error("invalid R value");
      }
      std::cout << "Green (G): ";
      std::cin >> g;
      if (g < 0 || g > 255) {
        throw std::runtime_error("invalid G value");
      }
      std::cout << "Blue (B): ";
      std::cin >> b;
      if (b < 0 || b > 255) {
        throw std::runtime_error("invalid B value");
      }

      std::cout
          << "\nSet priority (higher value = higher priority, default=6): ";
      std::cin >> priority;
      if (priority < 0) {
        throw std::runtime_error("invalid priority value");
      }

      std::string reset_input;
      std::cout << "Reset priority after command? (y/n, default=n): ";
      std::cin >> reset_input;
      std::transform(reset_input.begin(), reset_input.end(),
                     reset_input.begin(), ::tolower);
      reset_priority = (reset_input == "true" || reset_input == "1" ||
                        reset_input == "yes" || reset_input == "y");
    }

    if (client_node) {
      client_node->send_request(led_mode, r, g, b, priority, reset_priority);
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
