#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/srv/set_rgb_strip.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <signal.h>

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
    client_ = this->create_client<aimdk_msgs::srv::SetRgbStrip>(
        "/aimdk_5Fmsgs/srv/SetRgbStrip");
    RCLCPP_INFO(this->get_logger(), "✅ SetRgbStrip client node created.");

    while (!client_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "⏳ Service unavailable, waiting...");
    }
    RCLCPP_INFO(this->get_logger(),
                "🟢 Service available, ready to send request.");
  }

  bool send_request(uint8_t led_strip_mode) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetRgbStrip::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->led_strip_mode = led_strip_mode;

      RCLCPP_INFO(this->get_logger(),
                  "📨 Sending SetRgbStrip request: led_strip_mode=%u",
                  static_cast<unsigned int>(request->led_strip_mode));

      const std::chrono::milliseconds timeout(2000);
      request->request.header.stamp = this->now();
      auto future = client_->async_send_request(request);
      auto retcode = rclcpp::spin_until_future_complete(
          shared_from_this(), future, timeout);
      if (retcode != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(),
                    "❌ SetRgbStrip service timeout after %ld ms.",
                    timeout.count());
        return false;
      }

      auto response = future.get();
      const auto code = response->response.header.code;
      const auto status = response->response.status.value;
      RCLCPP_INFO(this->get_logger(),
                  "Response: code=%ld, status=%d, message=%s, result=%u",
                  code, status, response->response.message.c_str(),
                  static_cast<unsigned int>(response->result));

      if (code == 0 || status == aimdk_msgs::msg::CommonState::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✅ SetRgbStrip request accepted.");
        return true;
      }

      RCLCPP_ERROR(this->get_logger(), "❌ SetRgbStrip request failed.");
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

private:
  rclcpp::Client<aimdk_msgs::srv::SetRgbStrip>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    uint8_t led_strip_mode =
        aimdk_msgs::srv::SetRgbStrip::Request::LED_WHITE_ON;

    int mode_input = 0;
    std::cout << "Enter led_strip_mode (default "
              << static_cast<int>(led_strip_mode) << "): ";
    std::cin >> mode_input;
    led_strip_mode = static_cast<uint8_t>(mode_input);

    g_node = std::make_shared<PlayLightsClient>();
    auto client = std::dynamic_pointer_cast<PlayLightsClient>(g_node);
    bool ok = false;
    if (client) {
      ok = client->send_request(led_strip_mode);
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
