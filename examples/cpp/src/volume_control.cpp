/**
 * @file volume_control.cpp
 * @brief Volume Control Demo Example Script
 * 
 * @description
 *   This script demonstrates how to control robot audio volume and mute settings using TTS services.
 *   Includes volume adjustment, mute toggle, and TTS playback demonstration.
 *   Note: If a volume setting step fails, the script will continue to the next step. Please check the logs for any errors.
 *   The original volume is recorded at startup and automatically restored when the demo finishes or Ctrl+C is pressed.
 * 
 * @prerequisites
 *   - Robot TTS service must be running
 *   - Audio output device must be working properly
 *   - Volume and mute services must be available
 * 
 * @usage
 *   colcon build --packages-select aimdk_examples_cpp
 *   ros2 run aimdk_examples_cpp volume_control
 * 
 * @example
 *   ros2 run aimdk_examples_cpp volume_control
 * 
 * @parameters
 *   - None
 */

#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/tts_priority_level.hpp"
#include "aimdk_msgs/srv/get_mute.hpp"
#include "aimdk_msgs/srv/get_volume.hpp"
#include "aimdk_msgs/srv/play_tts.hpp"
#include "aimdk_msgs/srv/set_mute.hpp"
#include "aimdk_msgs/srv/set_volume.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <utility>

using namespace std::chrono_literals;

namespace
{

// CommonState reason 字段对应的中文描述
const std::unordered_map<uint32_t, std::string> kReasonDescriptions = {
    {0, "无错误"},
    {1, "开箱状态中"},
    {2, "开机自检中"},
    {3, "关机状态中"},
    {4, "当前形态不支持"},
    {5, "低电量限制"},
    {6, "正在充电中"},
    {7, "动作不在白名单"},
    {8, "HDS故障"},
    {9, "当前模式不支持"},
    {10, "前方有障碍物"},
    {11, "后方有障碍物"},
    {12, "左方有障碍物"},
    {13, "右方有障碍物"},
    {14, "上方有障碍物"}
};

std::string GetReasonDescription(uint32_t reason) {
  auto it = kReasonDescriptions.find(reason);
  if (it != kReasonDescriptions.end()) {
    return it->second;
  }
  return "未知原因(" + std::to_string(reason) + ")";
}

constexpr char kTtsText[] =
  "大家好，我是启元机器人。现在为你演示音频控制示例。"
  "接下来，我会先持续播报一段介绍内容，在播报过程中，系统会依次执行设置音量、查询音量、设置静音和查询静音等操作。"
  "你将听到默认音量、音量调小、音量调大、开启静音以及取消静音这几个阶段的变化。"
  "如果整段流程能够顺利完成，就说明启元的TTS播放、音量控制和静音控制链路都已经正常工作。"
  "感谢你体验启元机器人的音频控制能力。";

constexpr auto kStepInterval        = 5s;  // Delay between demo steps
constexpr auto kServiceWaitInterval = 2s;  // Poll interval while waiting for a service
constexpr auto kServiceCallTimeout  = 5s;  // Timeout for a single service request
constexpr int kMaxRetryCount = 3;          // Maximum retry attempts

}  // namespace

class VolumeControlClient : public rclcpp::Node
{
 public:
  VolumeControlClient() : Node("volume_control_client")
  {
    play_tts_client_ = this->create_client<aimdk_msgs::srv::PlayTts>(play_tts_service_);
    set_volume_client_ = this->create_client<aimdk_msgs::srv::SetVolume>(set_volume_service_);
    get_volume_client_ = this->create_client<aimdk_msgs::srv::GetVolume>(get_volume_service_);
    set_mute_client_ = this->create_client<aimdk_msgs::srv::SetMute>(set_mute_service_);
    get_mute_client_ = this->create_client<aimdk_msgs::srv::GetMute>(get_mute_service_);

    RCLCPP_INFO(this->get_logger(), "Volume control demo node created.");
  }

  bool wait_for_services()
  {
    return wait_for_service(play_tts_client_, play_tts_service_) &&
           wait_for_service(set_volume_client_, set_volume_service_) &&
           wait_for_service(get_volume_client_, get_volume_service_) &&
           wait_for_service(set_mute_client_, set_mute_service_) &&
           wait_for_service(get_mute_client_, get_mute_service_);
  }

  // ---- 信号处理 ----

  void handle_signal(int signal)
  {
    RCLCPP_INFO(this->get_logger(), "Received signal %d, will restore volume before shutdown.", signal);
    std::lock_guard<std::mutex> lock(state_mutex_);
    shutdown_requested_ = true;
  }

  bool is_shutdown_requested() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return shutdown_requested_;
  }

  // ---- 业务逻辑（同步，在 main 中调用） ----

  bool run_demo()
  {
    RCLCPP_INFO(this->get_logger(), "Volume control demo starts automatically.");
    RCLCPP_INFO(this->get_logger(),
                "Volume note: on this device a larger value means a smaller "
                "actual volume.");

    // Get initial volume to restore later (optional)
    has_original_volume_ = false;
    auto get_init_response = call_service<aimdk_msgs::srv::GetVolume>(
      get_volume_client_, build_get_volume_request(), "GetVolume");
    if (get_init_response) {
      original_volume_ = get_init_response->audio_volume;
      has_original_volume_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial volume: %u", original_volume_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get initial volume, will skip restoration.");
    }

    if (!play_tts()) {
      RCLCPP_ERROR(this->get_logger(), "PlayTts failed.");
      return false;
    }

    // 定义演示步骤
    struct Step {
      const char *name;
      std::function<bool()> func;
    };
    std::vector<Step> steps = {
      {"设为默认音量", [this]() { return execute_volume_step(30, "设为默认音量"); }},
      {"音量调大",     [this]() { return execute_volume_step(40, "音量调大"); }},
      {"音量调小",     [this]() { return execute_volume_step(20, "音量调小"); }},
      {"设置静音",     [this]() { return execute_mute_step(true, "设置静音"); }},
      {"取消静音",     [this]() { return execute_mute_step(false, "取消静音"); }},
    };

    RCLCPP_INFO(this->get_logger(),
                "TTS accepted. The first control step will run in %ld seconds.",
                kStepInterval.count());

    for (const auto &step : steps) {
      if (is_shutdown_requested()) { break; }
      std::this_thread::sleep_for(kStepInterval);
      if (is_shutdown_requested()) { break; }
      if (!step.func()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Volume control step failed: %s (continuing to next step)", step.name);
      }
    }

    // Restore original volume if available (normal finish or Ctrl+C)
    if (has_original_volume_) {
      RCLCPP_INFO(this->get_logger(), "Restoring original volume: %u", original_volume_);
      if (!execute_volume_step(original_volume_, "恢复原始音量")) {
        RCLCPP_WARN(this->get_logger(), "Failed to restore original volume.");
      }
      RCLCPP_INFO(this->get_logger(),
                  "Volume control demo finished. Volume restored to %u, "
                  "mute=false. (Check logs for any failed steps)", original_volume_);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Volume control demo finished. Volume restoration skipped "
                  "(initial volume unknown). (Check logs for any failed steps)");
    }
    return true;
  }

  bool restore_original_volume()
  {
    if (!has_original_volume_) {
      return true;
    }

    RCLCPP_INFO(this->get_logger(), "Restoring original volume: %u", original_volume_);

    auto set_request = build_set_volume_request(original_volume_);
    auto future = set_volume_client_->async_send_request(set_request);
    auto result = rclcpp::spin_until_future_complete(
      shared_from_this(), future, 5s);

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      try {
        auto response = future.get();
        if (response) {
          RCLCPP_INFO(this->get_logger(),
                      "Volume restored to %u on interrupt.", original_volume_);
          return true;
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "SetVolume failed: %s", e.what());
      }
    }

    RCLCPP_WARN(this->get_logger(),
                "Failed to restore original volume on interrupt.");
    return false;
  }

 private:
  bool wait_for_service(const rclcpp::ClientBase::SharedPtr &client, const std::string &service_name)
  {
    while (!client->wait_for_service(kServiceWaitInterval)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service: %s", service_name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Service available: %s", service_name.c_str());
    return true;
  }

  // ---- 通用服务调用（使用 spin_until_future_complete，边 spin 边等） ----

  template <typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> call_service(
    const typename rclcpp::Client<ServiceT>::SharedPtr &client,
    const std::shared_ptr<typename ServiceT::Request> &request,
    const char *service_name)
  {
    for (int i = 0; i < kMaxRetryCount; ++i) {
      auto future = client->async_send_request(request);
      auto result = rclcpp::spin_until_future_complete(
        shared_from_this(), future, kServiceCallTimeout);

      if (result == rclcpp::FutureReturnCode::SUCCESS) {
        try {
          auto response = future.get();
          if (response) { return response; }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "%s failed: %s", service_name, e.what());
        }
      }

      RCLCPP_INFO(this->get_logger(),
                  "%s attempt %d/%d timed out, retrying...",
                  service_name, i + 1, kMaxRetryCount);
      std::this_thread::sleep_for(200ms);
    }

    RCLCPP_ERROR(this->get_logger(),
                 "%s timed out after %d attempts (%ld s each).",
                 service_name, kMaxRetryCount, kServiceCallTimeout.count());
    return nullptr;
  }

  // ---- 请求构建 ----

  aimdk_msgs::msg::CommonRequest build_common_request()
  {
    aimdk_msgs::msg::CommonRequest req;
    req.header.stamp = this->now();
    return req;
  }

  std::shared_ptr<aimdk_msgs::srv::SetVolume::Request>
  build_set_volume_request(std::uint32_t volume)
  {
    auto req = std::make_shared<aimdk_msgs::srv::SetVolume::Request>();
    req->request = build_common_request();
    req->audio_volume = volume;
    return req;
  }

  std::shared_ptr<aimdk_msgs::srv::GetVolume::Request>
  build_get_volume_request()
  {
    auto req = std::make_shared<aimdk_msgs::srv::GetVolume::Request>();
    req->request = build_common_request();
    return req;
  }

  // ---- PlayTts ----

  bool play_tts()
  {
    auto request = std::make_shared<aimdk_msgs::srv::PlayTts::Request>();
    request->header = build_common_request();
    request->tts_req.text = kTtsText;
    request->tts_req.domain = tts_domain_;
    request->tts_req.trace_id = tts_trace_id_;
    request->tts_req.is_interrupted = true;
    request->tts_req.priority_level.value =
      aimdk_msgs::msg::TtsPriorityLevel::INTERACTION_L6;

    RCLCPP_INFO(this->get_logger(),
                "Sending PlayTts request. domain=%s trace_id=%s",
                request->tts_req.domain.c_str(), request->tts_req.trace_id.c_str());
    RCLCPP_INFO(this->get_logger(), "TTS text: %s", request->tts_req.text.c_str());

    auto response = call_service<aimdk_msgs::srv::PlayTts>(
      play_tts_client_, request, "PlayTts");
    if (!response) { return false; }

    RCLCPP_INFO(this->get_logger(),
                "PlayTts response: code=%ld status=%d is_success=%d "
                "error_message=%s",
                response->header.header.code, response->header.status.value,
                static_cast<int>(response->tts_resp.is_success),
                response->tts_resp.error_message.c_str());
    
    // 获取失败原因
    uint32_t reason = response->header.status.reason;
    if (reason > 0) {
      std::string reason_desc = GetReasonDescription(reason);
      RCLCPP_WARN(this->get_logger(), "PlayTts rejected: reason=%u - %s", reason, reason_desc.c_str());
    }

    return response->tts_resp.is_success;
  }

  // ---- Volume Step ----

  bool execute_volume_step(std::uint32_t target_volume, const char *action_message)
  {
    RCLCPP_INFO(this->get_logger(),
                "Running volume step. target=%u action=%s", target_volume, action_message);

    auto set_response = call_service<aimdk_msgs::srv::SetVolume>(
      set_volume_client_, build_set_volume_request(target_volume), "SetVolume");
    if (!set_response) { return false; }

    RCLCPP_INFO(this->get_logger(),
                "SetVolume response: code=%ld status=%d audio_volume=%u",
                set_response->response.header.code,
                set_response->response.status.value,
                set_response->audio_volume);
    
    // 获取失败原因
    {
      uint32_t reason = set_response->response.status.reason;
      if (reason > 0) {
        std::string reason_desc = GetReasonDescription(reason);
        RCLCPP_WARN(this->get_logger(), "SetVolume rejected: reason=%u - %s", reason, reason_desc.c_str());
      }
    }

    auto get_response = call_service<aimdk_msgs::srv::GetVolume>(
      get_volume_client_, build_get_volume_request(), "GetVolume");
    if (!get_response) { return false; }

    RCLCPP_INFO(this->get_logger(),
                "GetVolume response: code=%ld status=%d audio_volume=%u",
                get_response->response.header.code,
                get_response->response.status.value,
                get_response->audio_volume);
    
    // 获取失败原因
    {
      uint32_t reason = get_response->response.status.reason;
      if (reason > 0) {
        std::string reason_desc = GetReasonDescription(reason);
        RCLCPP_WARN(this->get_logger(), "GetVolume rejected: reason=%u - %s", reason, reason_desc.c_str());
      }
    }

    if (get_response->audio_volume != target_volume) {
      RCLCPP_ERROR(this->get_logger(),
                   "Volume verification failed. target=%u actual=%u",
                   target_volume, get_response->audio_volume);
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "%s，确认当前音量=%u", action_message, get_response->audio_volume);
    return true;
  }

  // ---- Mute Step ----

  bool execute_mute_step(bool target_mute, const char *action_message)
  {
    RCLCPP_INFO(this->get_logger(),
                "Running mute step. target=%d action=%s",
                static_cast<int>(target_mute), action_message);

    // Call GetMute after SetMute to verify the result.
    auto set_request                  = std::make_shared<aimdk_msgs::srv::SetMute::Request>();
    set_request->request              = build_common_request();
    set_request->is_mute              = target_mute;

    auto set_response = call_service<aimdk_msgs::srv::SetMute>(
      set_mute_client_, set_request, "SetMute");
    if (!set_response) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "SetMute response: code=%ld status=%d is_mute=%d "
                "(final result is verified by GetMute)",
                set_response->response.header.code,
                set_response->response.status.value,
                static_cast<int>(set_response->is_mute));
    
    // 获取失败原因
    {
      uint32_t reason = set_response->response.status.reason;
      if (reason > 0) {
        std::string reason_desc = GetReasonDescription(reason);
        RCLCPP_WARN(this->get_logger(), "SetMute rejected: reason=%u - %s", reason, reason_desc.c_str());
      }
    }

    auto get_request                  = std::make_shared<aimdk_msgs::srv::GetMute::Request>();
    get_request->request              = build_common_request();

    auto get_response = call_service<aimdk_msgs::srv::GetMute>(
      get_mute_client_, get_request, "GetMute");
    if (!get_response) { return false; }

    RCLCPP_INFO(this->get_logger(),
                "GetMute response: code=%ld status=%d is_mute=%d",
                get_response->response.header.code,
                get_response->response.status.value,
                static_cast<int>(get_response->is_mute));
    
    // 获取失败原因
    {
      uint32_t reason = get_response->response.status.reason;
      if (reason > 0) {
        std::string reason_desc = GetReasonDescription(reason);
        RCLCPP_WARN(this->get_logger(), "GetMute rejected: reason=%u - %s", reason, reason_desc.c_str());
      }
    }

    if (get_response->is_mute != target_mute) {
      RCLCPP_ERROR(this->get_logger(),
                   "Mute verification failed. target=%d actual=%d",
                   static_cast<int>(target_mute),
                   static_cast<int>(get_response->is_mute));
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "%s，确认当前静音状态=%d", action_message,
                static_cast<int>(get_response->is_mute));
    return true;
  }

  // ---- 成员变量 ----

  const std::string play_tts_service_   = "/aimdk_5Fmsgs/srv/PlayTts";
  const std::string set_volume_service_ = "/aimdk_5Fmsgs/srv/SetVolume";
  const std::string get_volume_service_ = "/aimdk_5Fmsgs/srv/GetVolume";
  const std::string set_mute_service_   = "/aimdk_5Fmsgs/srv/SetMute";
  const std::string get_mute_service_   = "/aimdk_5Fmsgs/srv/GetMute";
  const std::string tts_domain_         = "volume_control_demo";
  const std::string tts_trace_id_       = "volume_control_demo_trace";

  rclcpp::Client<aimdk_msgs::srv::PlayTts>::SharedPtr play_tts_client_;
  rclcpp::Client<aimdk_msgs::srv::SetVolume>::SharedPtr set_volume_client_;
  rclcpp::Client<aimdk_msgs::srv::GetVolume>::SharedPtr get_volume_client_;
  rclcpp::Client<aimdk_msgs::srv::SetMute>::SharedPtr set_mute_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMute>::SharedPtr get_mute_client_;

  mutable std::mutex state_mutex_;
  bool shutdown_requested_ = false;
  std::uint32_t original_volume_ = 30;
  bool has_original_volume_ = false;
};

std::shared_ptr<VolumeControlClient> g_node = nullptr;

// 修复 ROS2 中 Ctrl+C 后优先触发 shutdown 的问题
// 自定义信号处理器只设标志，不做服务调用
void signal_handler(int signal)
{
  if (g_node) {
    g_node->handle_signal(signal);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  g_node = std::make_shared<VolumeControlClient>();
  int exit_code = 0;

  try {
    if (!g_node->wait_for_services()) {
      RCLCPP_ERROR(rclcpp::get_logger("volume_control_main"),
                   "Failed to initialize volume control demo.");
      exit_code = 1;
    } else {
      g_node->run_demo();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("volume_control_main"),
                 "Program exited with exception: %s", e.what());
    exit_code = 1;
  }

  // Ctrl+C 中断时：上下文仍有效，在此恢复音量
  if (g_node && g_node->is_shutdown_requested()) {
    g_node->restore_original_volume();
  }

  g_node.reset();
  rclcpp::shutdown();
  return exit_code;
}
