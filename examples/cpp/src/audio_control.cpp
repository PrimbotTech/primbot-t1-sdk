#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/tts_priority_level.hpp"
#include "aimdk_msgs/srv/get_mute.hpp"
#include "aimdk_msgs/srv/get_volume.hpp"
#include "aimdk_msgs/srv/play_tts.hpp"
#include "aimdk_msgs/srv/set_mute.hpp"
#include "aimdk_msgs/srv/set_volume.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace {

constexpr char kTtsText[] =
    "大家好，我是启元机器人Q1。现在为你演示音频控制示例。"
    "接下来，我会先持续播报一段介绍内容，在播报过程中，系统会依次执行设置音量、查询音量、设置静音和查询静音等操作。"
    "你将听到默认音量、音量调小、音量调大、开启静音以及取消静音这几个阶段的变化。"
    "如果整段流程能够顺利完成，就说明启元的TTS播放、音量控制和静音控制链路都已经正常工作。"
    "感谢你体验启元机器人Q1的音频控制能力。";

constexpr auto kStepInterval = 5s;
constexpr auto kServiceWaitInterval = 2s;
constexpr auto kServiceCallTimeout = 5s;

constexpr std::size_t kTotalSteps = 5;

} // namespace

class VolumeControlClient : public rclcpp::Node {
public:
  VolumeControlClient() : Node("volume_control_client") {
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    play_tts_client_ = this->create_client<aimdk_msgs::srv::PlayTts>(
        play_tts_service_, rmw_qos_profile_services_default, callback_group_);
    set_volume_client_ = this->create_client<aimdk_msgs::srv::SetVolume>(
        set_volume_service_, rmw_qos_profile_services_default, callback_group_);
    get_volume_client_ = this->create_client<aimdk_msgs::srv::GetVolume>(
        get_volume_service_, rmw_qos_profile_services_default, callback_group_);
    set_mute_client_ = this->create_client<aimdk_msgs::srv::SetMute>(
        set_mute_service_, rmw_qos_profile_services_default, callback_group_);
    get_mute_client_ = this->create_client<aimdk_msgs::srv::GetMute>(
        get_mute_service_, rmw_qos_profile_services_default, callback_group_);

    startup_timer_ = this->create_wall_timer(
        200ms, std::bind(&VolumeControlClient::start_demo, this),
        callback_group_);

    RCLCPP_INFO(this->get_logger(),
                "Volume control demo node created. Waiting to start.");
  }

  bool initialize() {
    return wait_for_service(play_tts_client_, play_tts_service_) &&
           wait_for_service(set_volume_client_, set_volume_service_) &&
           wait_for_service(get_volume_client_, get_volume_service_) &&
           wait_for_service(set_mute_client_, set_mute_service_) &&
           wait_for_service(get_mute_client_, get_mute_service_);
  }

  void request_stop(int exit_code) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (shutdown_requested_) {
        if (exit_code_ == 0 && exit_code != 0) {
          exit_code_ = exit_code;
        }
      } else {
        shutdown_requested_ = true;
        if (exit_code_ == 0 || exit_code != 0) {
          exit_code_ = exit_code;
        }
      }
    }

    if (startup_timer_) {
      startup_timer_->cancel();
    }
    if (step_timer_) {
      step_timer_->cancel();
    }
  }

  int exit_code() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return exit_code_;
  }

  void handle_signal(int signal) {
    RCLCPP_INFO(this->get_logger(), "Received signal %d, shutting down...",
                signal);
    request_stop(signal);
  }

private:
  bool wait_for_service(const rclcpp::ClientBase::SharedPtr &client,
                        const std::string &service_name) {
    while (!client->wait_for_service(kServiceWaitInterval)) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service: %s",
                  service_name.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Service available: %s",
                service_name.c_str());
    return true;
  }

  template <typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> call_service(
      const typename rclcpp::Client<ServiceT>::SharedPtr &client,
      const std::shared_ptr<typename ServiceT::Request> &request,
      const char *service_name) {
    auto future = client->async_send_request(request);
    if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "%s timed out after %ld ms.",
                   service_name, kServiceCallTimeout.count());
      return nullptr;
    }
    return future.get();
  }

  void start_demo() {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (shutdown_requested_ || started_) {
        return;
      }
      started_ = true;
    }

    startup_timer_->cancel();

    RCLCPP_INFO(this->get_logger(),
                "Volume control demo starts automatically.");
    RCLCPP_INFO(this->get_logger(),
                "Volume note: on this device a larger value means a smaller "
                "actual volume.");

    if (!play_tts()) {
      fail_and_shutdown("PlayTts failed.");
      return;
    }

    step_timer_ = this->create_wall_timer(
        kStepInterval, std::bind(&VolumeControlClient::run_step, this),
        callback_group_);

    RCLCPP_INFO(this->get_logger(),
                "TTS accepted. The first control step will run in %ld seconds.",
                kStepInterval.count());
  }

  bool play_tts() {
    auto request = std::make_shared<aimdk_msgs::srv::PlayTts::Request>();
    request->header = aimdk_msgs::msg::CommonRequest();
    request->header.header.stamp = this->now();
    request->tts_req.text = kTtsText;
    request->tts_req.domain = tts_domain_;
    request->tts_req.trace_id = tts_trace_id_;
    request->tts_req.is_interrupted = true;
    request->tts_req.priority_weight = 0;
    request->tts_req.priority_level.value =
        aimdk_msgs::msg::TtsPriorityLevel::INTERACTION_L6;

    RCLCPP_INFO(this->get_logger(),
                "Sending PlayTts request. domain=%s trace_id=%s",
                request->tts_req.domain.c_str(),
                request->tts_req.trace_id.c_str());
    RCLCPP_INFO(this->get_logger(), "TTS text: %s",
                request->tts_req.text.c_str());

    auto response =
        call_service<aimdk_msgs::srv::PlayTts>(play_tts_client_, request,
                                               "PlayTts");
    if (!response) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "PlayTts response: code=%ld status=%d is_success=%d "
                "error_message=%s",
                response->header.header.code, response->header.status.value,
                static_cast<int>(response->tts_resp.is_success),
                response->tts_resp.error_message.c_str());

    if (!response->tts_resp.is_success) {
      return false;
    }

    return true;
  }

  void run_step() {
    std::size_t current_step = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (shutdown_requested_) {
        return;
      }
      if (step_in_progress_) {
        RCLCPP_WARN(this->get_logger(),
                    "Previous control step is still running, skip this tick.");
        return;
      }
      if (step_index_ >= kTotalSteps) {
        return;
      }
      step_in_progress_ = true;
      current_step = step_index_;
    }

    bool ok = false;
    switch (current_step) {
    case 0:
      ok = execute_volume_step(30, "设为默认音量");
      break;
    case 1:
      ok = execute_volume_step(40, "音量调小");
      break;
    case 2:
      ok = execute_volume_step(20, "音量调大");
      break;
    case 3:
      ok = execute_mute_step(true, "设置静音");
      break;
    case 4:
      ok = execute_mute_step(false, "取消静音");
      break;
    default:
      ok = false;
      break;
    }

    bool is_done = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      step_in_progress_ = false;
      if (shutdown_requested_) {
        return;
      }
      if (ok) {
        ++step_index_;
        is_done = step_index_ >= kTotalSteps;
      }
    }

    if (!ok) {
      fail_and_shutdown("Volume control step failed.");
      return;
    }

    if (is_done) {
      RCLCPP_INFO(this->get_logger(),
                  "Volume control demo finished. Final state: volume=20, "
                  "mute=false.");
      request_stop(0);
      rclcpp::shutdown();
    }
  }

  bool execute_volume_step(std::uint32_t target_volume,
                           const char *action_message) {
    RCLCPP_INFO(this->get_logger(),
                "Running volume step. target=%u action=%s",
                target_volume, action_message);

    auto set_request = std::make_shared<aimdk_msgs::srv::SetVolume::Request>();
    set_request->request = aimdk_msgs::msg::CommonRequest();
    set_request->request.header.stamp = this->now();
    set_request->audio_volume = target_volume;

    auto set_response = call_service<aimdk_msgs::srv::SetVolume>(
        set_volume_client_, set_request, "SetVolume");
    if (!set_response) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "SetVolume response: code=%ld status=%d audio_volume=%u",
                set_response->reponse.header.code,
                set_response->reponse.status.value,
                set_response->audio_volume);

    auto get_request = std::make_shared<aimdk_msgs::srv::GetVolume::Request>();
    get_request->request = aimdk_msgs::msg::CommonRequest();
    get_request->request.header.stamp = this->now();

    auto get_response = call_service<aimdk_msgs::srv::GetVolume>(
        get_volume_client_, get_request, "GetVolume");
    if (!get_response) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "GetVolume response: code=%ld status=%d audio_volume=%u",
                get_response->reponse.header.code,
                get_response->reponse.status.value,
                get_response->audio_volume);

    if (get_response->audio_volume != target_volume) {
      RCLCPP_ERROR(this->get_logger(),
                   "Volume verification failed. target=%u actual=%u",
                   target_volume, get_response->audio_volume);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "%s，确认当前音量=%u", action_message,
                get_response->audio_volume);
    return true;
  }

  bool execute_mute_step(bool target_mute, const char *action_message) {
    RCLCPP_INFO(this->get_logger(), "Running mute step. target=%d action=%s",
                static_cast<int>(target_mute), action_message);

    auto set_request = std::make_shared<aimdk_msgs::srv::SetMute::Request>();
    set_request->request = aimdk_msgs::msg::CommonRequest();
    set_request->request.header.stamp = this->now();
    set_request->is_mute = target_mute;

    auto set_response = call_service<aimdk_msgs::srv::SetMute>(
        set_mute_client_, set_request, "SetMute");
    if (!set_response) {
      return false;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "SetMute response: code=%ld status=%d is_mute=%d "
        "(final result is verified by GetMute)",
        set_response->reponse.header.code,
        set_response->reponse.status.value,
        static_cast<int>(set_response->is_mute));

    auto get_request = std::make_shared<aimdk_msgs::srv::GetMute::Request>();
    get_request->request = aimdk_msgs::msg::CommonRequest();
    get_request->request.header.stamp = this->now();

    auto get_response = call_service<aimdk_msgs::srv::GetMute>(
        get_mute_client_, get_request, "GetMute");
    if (!get_response) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "GetMute response: code=%ld status=%d is_mute=%d",
                get_response->reponse.header.code,
                get_response->reponse.status.value,
                static_cast<int>(get_response->is_mute));

    if (get_response->is_mute != target_mute) {
      RCLCPP_ERROR(this->get_logger(),
                   "Mute verification failed. target=%d actual=%d",
                   static_cast<int>(target_mute),
                   static_cast<int>(get_response->is_mute));
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "%s，确认当前静音状态=%d", action_message,
                static_cast<int>(get_response->is_mute));
    return true;
  }

  void fail_and_shutdown(const char *reason) {
    RCLCPP_ERROR(this->get_logger(), "%s", reason);
    request_stop(1);
    rclcpp::shutdown();
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  const std::string play_tts_service_ = "/aimdk_5Fmsgs/srv/PlayTts";
  const std::string set_volume_service_ = "/aimdk_5Fmsgs/srv/SetVolume";
  const std::string get_volume_service_ = "/aimdk_5Fmsgs/srv/GetVolume";
  const std::string set_mute_service_ = "/aimdk_5Fmsgs/srv/SetMute";
  const std::string get_mute_service_ = "/aimdk_5Fmsgs/srv/GetMute";
  const std::string tts_domain_ = "volume_control_demo";
  const std::string tts_trace_id_ = "volume_control_demo_trace";
  rclcpp::Client<aimdk_msgs::srv::PlayTts>::SharedPtr play_tts_client_;
  rclcpp::Client<aimdk_msgs::srv::SetVolume>::SharedPtr set_volume_client_;
  rclcpp::Client<aimdk_msgs::srv::GetVolume>::SharedPtr get_volume_client_;
  rclcpp::Client<aimdk_msgs::srv::SetMute>::SharedPtr set_mute_client_;
  rclcpp::Client<aimdk_msgs::srv::GetMute>::SharedPtr get_mute_client_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  mutable std::mutex state_mutex_;
  std::size_t step_index_ = 0;
  bool started_ = false;
  bool step_in_progress_ = false;
  bool shutdown_requested_ = false;
  int exit_code_ = 0;
};

std::shared_ptr<VolumeControlClient> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    g_node->handle_signal(signal);
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    g_node = std::make_shared<VolumeControlClient>();
    if (!g_node->initialize()) {
      RCLCPP_ERROR(rclcpp::get_logger("volume_control_main"),
                   "Failed to initialize volume control demo.");
      const int exit_code = 1;
      g_node.reset();
      rclcpp::shutdown();
      return exit_code;
    }

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(g_node);
    executor.spin();

    const int exit_code = g_node ? g_node->exit_code() : 0;
    g_node.reset();
    rclcpp::shutdown();
    return exit_code;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("volume_control_main"),
                 "Program exited with exception: %s", e.what());
    return 1;
  }
}
