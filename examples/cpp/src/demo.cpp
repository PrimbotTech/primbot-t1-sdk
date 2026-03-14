#include <signal.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "aimdk_msgs/srv/play_emotion.hpp"
#include "aimdk_msgs/srv/play_tts.hpp"
#include "aimdk_msgs/srv/set_mc_action.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 1. 全局指针用于信号拦截
std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signal_handler(int signal)
{
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Received signal %d, shutting down...", signal);
    g_node.reset();
  }
  rclcpp::shutdown();
  exit(signal);
}

class DemoNode : public rclcpp::Node
{
 public:
  DemoNode() : Node("cpp_demo_node")
  {
    RCLCPP_INFO(this->get_logger(), "Demo Node Initialized.");
  }

  // 通用同步调用 Service 的模板函数
  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr call_service(
    const std::string& service_name,
    typename ServiceT::Request::SharedPtr request)
  {
    auto client = this->create_client<ServiceT>(service_name);

    if (!client->wait_for_service(3s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service %s.", service_name.c_str());
        return nullptr;
      }
      RCLCPP_ERROR(this->get_logger(), "Service %s not available after waiting.", service_name.c_str());
      return nullptr;
    }

    auto result_future = client->async_send_request(request);

    // 等待结果，设置超时防止死锁
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
      return result_future.get();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service %s or timeout occurred.", service_name.c_str());
      return nullptr;
    }
  }

  void run_demo()
  {
    RCLCPP_INFO(this->get_logger(), "Starting 启元机器人 aimdk_msgs C++ Demo...");

    // 1. 播放表情 (Interaction层 srv)
    RCLCPP_INFO(this->get_logger(), "[1/3] 播放表情(happy)...");
    auto emotion_req        = std::make_shared<aimdk_msgs::srv::PlayEmotion::Request>();
    emotion_req->emotion_id = aimdk_msgs::srv::PlayEmotion::Request::EMOTION_EYE_HAPPY;
    emotion_req->mode       = aimdk_msgs::srv::PlayEmotion::Request::EMOTION_MODE_ONCE;
    emotion_req->priority   = 10;
    auto emotion_resp       = this->call_service<aimdk_msgs::srv::PlayEmotion>("/interaction/play_emotion", emotion_req);
    if (emotion_resp) {
      RCLCPP_INFO(this->get_logger(), "命令发送成功: %s", (emotion_resp->success ? "true" : "false"));
    }

    // 2. 执行一个动作 (MC层 srv)
    RCLCPP_INFO(this->get_logger(), "[2/3] 执行动作(握手)...");
    auto action_req                  = std::make_shared<aimdk_msgs::srv::SetMcAction::Request>();
    action_req->command.action.value = aimdk_msgs::msg::McAction::QUADRUPED_LOCOMOTION_HANDSHAKE;
    action_req->command.action_desc  = "handshake demo from cpp";
    auto action_resp                 = this->call_service<aimdk_msgs::srv::SetMcAction>("/mc/set_action", action_req);
    if (action_resp) {
      RCLCPP_INFO(this->get_logger(), "动作命令发送完成");
    }

    // 3. 播放 TTS 语音 (Interaction层 srv)
    RCLCPP_INFO(this->get_logger(), "[3/3] 播放TTS语音...");
    auto tts_req                          = std::make_shared<aimdk_msgs::srv::PlayTts::Request>();
    tts_req->tts_req.text                 = "你好，我是启元机器人。C++ SDK 测试成功！";
    tts_req->tts_req.priority_level.value = aimdk_msgs::msg::TtsPriorityLevel::INTERACTION_L6;
    tts_req->tts_req.domain               = "sdk_demo_cpp";
    tts_req->tts_req.is_interrupted       = true;
    auto tts_resp                         = this->call_service<aimdk_msgs::srv::PlayTts>("/interaction/play_tts", tts_req);
    if (tts_resp) {
      RCLCPP_INFO(this->get_logger(), "TTS请求发送完成");
    }

    RCLCPP_INFO(this->get_logger(), "=== Demo 完成 ===");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  try {
    g_node         = std::make_shared<DemoNode>();
    auto demo_node = std::static_pointer_cast<DemoNode>(g_node);

    // 执行单次 Demo 逻辑
    demo_node->run_demo();

    // 如果需要持续运行，可以使用 spin
    // rclcpp::spin(g_node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
  }

  if (g_node) {
    g_node.reset();
  }
  rclcpp::shutdown();
  return 0;
}
