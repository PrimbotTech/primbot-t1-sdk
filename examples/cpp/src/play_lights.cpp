#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/srv/led_strip_command.hpp"
#include "aimdk_msgs/srv/set_neck_light.hpp"
#include "aimdk_msgs/msg/neck_light_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <signal.h>
#include <string>
#include <thread>
#include <unordered_map>

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
    led_client_ = this->create_client<aimdk_msgs::srv::LedStripCommand>(
        "/aimdk_5Fmsgs/srv/LedStripCommand");
    RCLCPP_INFO(this->get_logger(), "LedStripCommand client node created.");

    neck_client_ = this->create_client<aimdk_msgs::srv::SetNeckLight>(
        "/aimdk_5Fmsgs/srv/SetNeckLight");
    RCLCPP_INFO(this->get_logger(), "SetNeckLight client node created.");

    while (!led_client_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service unavailable, waiting...");
    }
    
    while (!neck_client_->wait_for_service(std::chrono::seconds(2))) {
      if (!rclcpp::ok()) {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for SetNeckLight service...");
    }
    
    RCLCPP_INFO(this->get_logger(),
                "All services available, ready to send request.");
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
      auto future = led_client_->async_send_request(request);
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
        future = led_client_->async_send_request(request);
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
            
      RCLCPP_ERROR(this->get_logger(),
                   "LedStripCommand failed. code=%ld status=%d msg=%s",
                   code, static_cast<int>(status_value),
                   response->header.message.c_str());
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
      return false;
    }
  }

  bool set_neck_light(bool enable, uint8_t brightness) {
    try {
      auto request = std::make_shared<aimdk_msgs::srv::SetNeckLight::Request>();
      request->request = aimdk_msgs::msg::CommonRequest();
      request->request.header.stamp = this->now();
      request->enable = enable;
      request->brightness = brightness;

      if (enable) {
        RCLCPP_INFO(this->get_logger(), "Sending SetNeckLight: enable=%s, brightness=%u%%",
                    enable ? "true" : "false", brightness);
      } else {
        RCLCPP_INFO(this->get_logger(), "Sending SetNeckLight: enable=%s",
                    enable ? "true" : "false");
      }

      // Retry mechanism: up to 3 attempts
      auto future = neck_client_->async_send_request(request);
      bool completed = false;
      
      for (int i = 0; i < kMaxRetryCount; ++i) {
        auto retcode = rclcpp::spin_until_future_complete(
            shared_from_this(), future, kServiceCallTimeout);
        
        if (retcode == rclcpp::FutureReturnCode::SUCCESS) {
          completed = true;
          break;
        }

        RCLCPP_INFO(this->get_logger(),
                    "SetNeckLight attempt %d/%d timed out, retrying...",
                    i + 1, kMaxRetryCount);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Re-send request for retry
        future = neck_client_->async_send_request(request);
      }

      if (!completed) {
        RCLCPP_ERROR(this->get_logger(),
                     "SetNeckLight service timeout.");
        return false;
      }

      auto response = future.get();
      if (!response) {
        RCLCPP_ERROR(this->get_logger(), "SetNeckLight call failed.");
        return false;
      }

      const auto code = response->header.header.code;
      if (code == 0) {
        RCLCPP_INFO(this->get_logger(), "SetNeckLight accepted.");
        return true;
      }

      RCLCPP_ERROR(this->get_logger(),
                   "SetNeckLight failed. code=%ld status=%d msg=%s",
                   code,
                   response->header.status.value,
                   response->header.header.message.c_str());
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in set_neck_light: %s", e.what());
      return false;
    }
  }

private:
  template<typename ServiceT>
  bool wait_for_future(
      rclcpp::Client<ServiceT> &client,
      typename rclcpp::Client<ServiceT>::SharedFuture future,
      const typename ServiceT::Request::SharedPtr &request) {
    for (int i = 0; i < kMaxRetryCount; ++i) {
      if (rclcpp::spin_until_future_complete(shared_from_this(), future, kServiceCallTimeout) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        return true;
      }
      RCLCPP_INFO(this->get_logger(), "Timeout %d/%d, retrying...", i + 1, kMaxRetryCount);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      future = client.async_send_request(request).future.share();
    }
    RCLCPP_ERROR(this->get_logger(), "Service timeout.");
    return false;
  }

private:
  rclcpp::Client<aimdk_msgs::srv::LedStripCommand>::SharedPtr led_client_;
  rclcpp::Client<aimdk_msgs::srv::SetNeckLight>::SharedPtr neck_client_;
};

class GetNeckLightStateSubscriber: public rclcpp::Node{
public:
   GetNeckLightStateSubscriber(): Node("get_neck_light_state_subscriber") {
    current_state_ = nullptr;
    subscription_ = this->create_subscription<aimdk_msgs::msg::NeckLightState>(
        "/aima/hal/neck_light/state", 10,
        std::bind(&GetNeckLightStateSubscriber::state_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Neck Light State Subscriber node created.");
  }

  aimdk_msgs::msg::NeckLightState::SharedPtr current_state_ = nullptr;

private:
  void state_callback(const aimdk_msgs::msg::NeckLightState::SharedPtr msg) {
    current_state_ = msg;
    if (msg->enable) {
      RCLCPP_INFO(this->get_logger(), "State updated: enable=%d, brightness=%d", 
                  msg->enable, msg->brightness);
    }
    if (!msg->enable) {
      RCLCPP_INFO(this->get_logger(), "State updated: enable=%d", msg->enable);
    }
  }

  rclcpp::Subscription<aimdk_msgs::msg::NeckLightState>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Print menu
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "  LED Lights Control Menu" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "\nSelect control mode:" << std::endl;
    std::cout << "  1. LED Strip Control" << std::endl;
    std::cout << "  2. Neck Light Control" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    int choice = 1;
    std::cout << "\nEnter choice (1-2, default 1): ";
    std::cin >> choice;
    
    g_node = std::make_shared<PlayLightsClient>();
    auto client = std::dynamic_pointer_cast<PlayLightsClient>(g_node);
    
    if (!client) {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create client");
      g_node.reset();
      rclcpp::shutdown();
      return 1;
    }
    
    bool ok = false;
    
    if (choice == 1) {
      // LED strip control
      uint8_t led_strip_mode =
          aimdk_msgs::srv::LedStripCommand::Request::LED_WHITE_ON;
      uint8_t r = 0;
      uint8_t g = 0;
      uint8_t b = 0;
      uint16_t period = 0;

      int mode_input;
      std::cout << "\nEnter led_strip_mode (default "
                << static_cast<int>(led_strip_mode) << "): ";
      if (std::cin >> mode_input) {
        led_strip_mode = static_cast<uint8_t>(mode_input);
      }

      if (led_strip_mode ==
          aimdk_msgs::srv::LedStripCommand::Request::LED_CUSTOM) {
        int channel_input = 0;
        int period_input = 1000;

        std::cout << "  Enter r (0-255, default 0): ";
        std::cin >> channel_input;
        r = static_cast<uint8_t>(channel_input);

        std::cout << "  Enter g (0-255, default 0): ";
        std::cin >> channel_input;
        g = static_cast<uint8_t>(channel_input);

        std::cout << "  Enter b (0-255, default 255): ";
        channel_input = 255;
        std::cin >> channel_input;
        b = static_cast<uint8_t>(channel_input);

        std::cout << "  Enter period(ms, default 1000): ";
        std::cin >> period_input;
        period = static_cast<uint16_t>(period_input);
      }

      ok = client->send_request(led_strip_mode, r, g, b, period);
      if (ok) {
        std::cout << "\n✓ LED strip control request sent" << std::endl;
      } else {
        std::cout << "\n✗ LED strip control request failed" << std::endl;
      }
    } else if (choice == 2) {
      // Neck light control
      // Create state subscriber to monitor light state
      auto state_subscriber = std::make_shared<GetNeckLightStateSubscriber>();
      
      // Spin briefly to receive initial state
      rclcpp::spin_some(state_subscriber);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      std::cout << "\n--- Neck Light Control ---" << std::endl;
      
      bool enable = true;
      std::string enable_input;
      std::cout << "  Enable (true/false, default true): ";
      std::cin >> enable_input;
      
      // Handle user input
      if (enable_input == "" || enable_input == "true" || enable_input == "1") {
        enable = true;
      } else if (enable_input == "false" || enable_input == "0") {
        enable = false;
      } else {
        std::cout << "\n✗ Invalid input, please use true/false" << std::endl;
        g_node.reset();
        rclcpp::shutdown();
        return 1;
      }
      
      // Only prompt for brightness when light is enabled
      int brightness = 50;
      if (enable) {
        std::cout << "  Brightness (0-100, default 50): ";
        std::cin >> brightness;
        
        // Validate brightness range
        if (brightness < 0 || brightness > 100) {
          std::cout << "\n✗ Brightness must be in range 0-100" << std::endl;
          g_node.reset();
          rclcpp::shutdown();
          return 1;
        }
      }
      
      ok = client->set_neck_light(enable, static_cast<uint8_t>(brightness));
      if (ok) {
        if (enable) {
          std::cout << "\n✓ Neck light enabled, brightness " << brightness << "%" << std::endl;
        } else {
          std::cout << "\n✓ Neck light disabled" << std::endl;
        }
        
        // Wait for state update and display
        std::cout << "\nWaiting for state update..." << std::endl;
        bool state_received = false;
        for (int i = 0; i < 5; ++i) {
          rclcpp::spin_some(state_subscriber);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          
          if (state_subscriber->current_state_) {
            auto state = state_subscriber->current_state_;
            if (state->enable) {
              std::cout << "Current state: enable=" << state->enable 
                       << ", brightness=" << static_cast<int>(state->brightness) << std::endl;
            } else {
              std::cout << "Current state: enable=" << state->enable << std::endl;
            }
            state_received = true;
            break;
          }
        }
        
        if (!state_received) {
          std::cout << "No state update received (topic may not be published)" << std::endl;
        }
      } else {
        std::cout << "\n✗ Neck light control request failed" << std::endl;
      }
    } else {
      std::cout << "\n✗ Invalid option" << std::endl;
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
