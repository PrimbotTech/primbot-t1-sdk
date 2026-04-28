#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/srv/led_strip_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <signal.h>
#include <thread>

constexpr int kMaxRetryCount = 3;
constexpr std::chrono::milliseconds kServiceCallTimeout(2000);

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
    client_ = this->create_client<aimdk_msgs::srv::LedStripCommand>(
        "/aimdk_5Fmsgs/srv/LedStripCommand");
    RCLCPP_INFO(this->get_logger(), "LedStripCommand client node created.");

    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "Service available, ready to send request.");
  }

  bool send_request(uint8_t led_strip_mode, uint8_t r, uint8_t g, uint8_t b,
                    uint16_t period) {
    try {
      auto request =
          std::make_shared<aimdk_msgs::srv::LedStripCommand::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->led_strip_mode = led_strip_mode;
      request->r = r;
      request->g = g;
      request->b = b;
      request->period = period;

      RCLCPP_INFO(this->get_logger(),
                  "Sending LedStripCommand request: led_strip_mode=%u, "
                  "r=%u, g=%u, b=%u, period=%u",
                  static_cast<unsigned int>(request->led_strip_mode),
                  static_cast<unsigned int>(request->r),
                  static_cast<unsigned int>(request->g),
                  static_cast<unsigned int>(request->b),
                  static_cast<unsigned int>(request->period));

      // Retry mechanism: up to 3 attempts
      request->request.header.stamp = this->now();
      auto future = client_->async_send_request(request);
      bool completed = false;
      
      for (int i = 0; i < kMaxRetryCount; ++i) {
        auto retcode = rclcpp::spin_until_future_complete(
            shared_from_this(), future, kServiceCallTimeout);
        
        if (retcode == rclcpp::FutureReturnCode::SUCCESS) {
          completed = true;
          break;
        }

        RCLCPP_INFO(this->get_logger(),
                    "LedStripCommand attempt %d/%d timed out, retrying...",
                    i + 1, kMaxRetryCount);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Re-send request for retry
        future = client_->async_send_request(request);
      }

      if (!completed) {
        RCLCPP_ERROR(this->get_logger(),
                     "LedStripCommand service timeout.");
        return false;
      }

      auto response = future.get();
      if (!response) {
        RCLCPP_ERROR(this->get_logger(),
                     "LedStripCommand service call failed.");
        return false;
      }
      
      const auto code = response->header.header.code;  
      const auto status_value = response->header.status.value; 
      RCLCPP_INFO(this->get_logger(),
                  "Response: code=%ld, status_value=%d", code,
                  static_cast<int>(status_value));

      if (code == 0 && status_value == 1) {  // SUCCESS = 1
        RCLCPP_INFO(this->get_logger(),
                    "LedStripCommand request accepted.");
        return true;
      }

      RCLCPP_ERROR(this->get_logger(), "LedStripCommand request failed.");
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  rclcpp::Client<aimdk_msgs::srv::LedStripCommand>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    uint8_t led_strip_mode =
        aimdk_msgs::srv::LedStripCommand::Request::LED_WHITE_ON;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint16_t period = 0;

    int mode_input = 0;
    std::cout << "Enter led_strip_mode (default "
              << static_cast<int>(led_strip_mode) << "): ";
    std::cin >> mode_input;
    led_strip_mode = static_cast<uint8_t>(mode_input);

    if (led_strip_mode ==
        aimdk_msgs::srv::LedStripCommand::Request::LED_CUSTOM) {
      int channel_input = 0;
      int period_input = 1000;

      std::cout << "Enter r (default 0): ";
      std::cin >> channel_input;
      r = static_cast<uint8_t>(channel_input);

      std::cout << "Enter g (default 0): ";
      std::cin >> channel_input;
      g = static_cast<uint8_t>(channel_input);

      std::cout << "Enter b (default 255): ";
      channel_input = 255;
      std::cin >> channel_input;
      b = static_cast<uint8_t>(channel_input);

      std::cout << "Enter period(ms, default 1000): ";
      std::cin >> period_input;
      period = static_cast<uint16_t>(period_input);
    }

    g_node = std::make_shared<PlayLightsClient>();
    auto client = std::dynamic_pointer_cast<PlayLightsClient>(g_node);
    bool ok = false;
    if (client) {
      ok = client->send_request(led_strip_mode, r, g, b, period);
    }

    g_node.reset();
    rclcpp::shutdown();
    return ok ? 0 : 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
