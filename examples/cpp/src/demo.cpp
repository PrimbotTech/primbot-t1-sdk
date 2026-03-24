#include "aimdk_msgs/msg/common_request.hpp"
#include "aimdk_msgs/msg/common_state.hpp"
#include "aimdk_msgs/msg/mc_preset_motion.hpp"
#include "aimdk_msgs/msg/request_header.hpp"
#include "aimdk_msgs/msg/touch_state.hpp"
#include "aimdk_msgs/msg/tts_priority_level.hpp"
#include "aimdk_msgs/srv/play_emotion.hpp"
#include "aimdk_msgs/srv/play_tts.hpp"
#include "aimdk_msgs/srv/led_strip_command.hpp"
#include "aimdk_msgs/srv/set_mc_preset_motion.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <csignal>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

constexpr char kTouchTopic[] = "/aima/hal/touch/state";
constexpr char kLedStripCommandService[] = "/aimdk_5Fmsgs/srv/LedStripCommand";
constexpr char kPlayEmotionService[] = "/aimdk_5Fmsgs/srv/PlayEmotion";
constexpr char kPlayTtsService[] = "/aimdk_5Fmsgs/srv/PlayTts";
constexpr char kSetPresetMotionService[] = "/aimdk_5Fmsgs/srv/SetMcPresetMotion";

constexpr char kOpeningText[] = "你好，欢迎体验 SDK 综合演示。";
constexpr char kClosingText[] = "要不要再来一次。";
constexpr char kTtsDomain[] = "sdk_demo_cpp";
constexpr char kTtsTraceId[] = "swipe_heart_demo";

constexpr int32_t kEmotionId = 90;
constexpr int32_t kEmotionLoopCount = 3;

constexpr auto kServiceWaitInterval = 2s;
constexpr auto kServiceCallTimeout = 2s;
constexpr auto kTtsPlaybackBuffer = 300ms;
constexpr auto kPresetMotionDisplayDuration = 8s;
constexpr auto kCooldownDuration = 1500ms;

} // namespace

enum class DemoState {
  Cooldown,
  Idle,
  Busy,
};

class SwipeHeartDemo : public rclcpp::Node {
public:
  SwipeHeartDemo() : Node("swipe_heart_demo") {
    led_strip_client_ =
        this->create_client<aimdk_msgs::srv::LedStripCommand>(
            kLedStripCommandService);
    play_emotion_client_ =
        this->create_client<aimdk_msgs::srv::PlayEmotion>(kPlayEmotionService);
    play_tts_client_ =
        this->create_client<aimdk_msgs::srv::PlayTts>(kPlayTtsService);
    set_preset_motion_client_ =
        this->create_client<aimdk_msgs::srv::SetMcPresetMotion>(
            kSetPresetMotionService);

    touch_sub_ = this->create_subscription<aimdk_msgs::msg::TouchState>(
        kTouchTopic, rclcpp::SensorDataQoS(),
        std::bind(&SwipeHeartDemo::touch_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Swipe heart demo node created, waiting for services.");
  }

  ~SwipeHeartDemo() override {
    request_stop();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  bool initialize() {
    if (!wait_for_required_services()) {
      return false;
    }

    start_worker();

    if (!set_led_mode(aimdk_msgs::srv::LedStripCommand::Request::LED_WHITE_ON)) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to set idle light during startup, continue anyway.");
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (stop_requested_) {
        return false;
      }
      state_ = DemoState::Idle;
    }

    if (!play_emotion_client_->wait_for_service(0s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Optional service not ready at startup: %s. Emotion playback "
                  "will be skipped until the service becomes available.",
                  kPlayEmotionService);
    } else {
      RCLCPP_INFO(this->get_logger(), "Optional service ready: %s",
                  kPlayEmotionService);
    }

    RCLCPP_INFO(this->get_logger(),
                "Demo ready. Slide left or right on the touch pad to trigger.");
    return true;
  }

  void request_stop() {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stop_requested_ = true;
      has_pending_trigger_ = false;
    }
    state_cv_.notify_all();
  }

  void handle_signal(int signal) {
    RCLCPP_INFO(this->get_logger(), "Received signal %d, shutting down...",
                signal);
    request_stop();
  }

private:
  void start_worker() {
    if (worker_thread_.joinable()) {
      return;
    }

    // The worker serializes the demo flow so touch callbacks stay non-blocking.
    worker_thread_ = std::thread(&SwipeHeartDemo::worker_loop, this);
  }

  void touch_callback(const aimdk_msgs::msg::TouchState::SharedPtr msg) {
    if (!is_target_trigger(msg->event_type)) {
      return;
    }

    const char *trigger_name = trigger_name_from_event(msg->event_type);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (stop_requested_) {
        return;
      }

      if (state_ != DemoState::Idle) {
        RCLCPP_INFO(this->get_logger(),
                    "Ignore trigger %s while demo is %s.", trigger_name,
                    state_name(state_));
        return;
      }

      state_ = DemoState::Busy;
      pending_trigger_ = msg->event_type;
      has_pending_trigger_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "Accepted trigger: %s", trigger_name);
    state_cv_.notify_one();
  }

  void worker_loop() {
    while (rclcpp::ok()) {
      uint8_t trigger = aimdk_msgs::msg::TouchState::TOUCH_EVENT_NONE;

      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_cv_.wait(lock, [this] {
          return stop_requested_ || has_pending_trigger_;
        });

        if (stop_requested_ || !rclcpp::ok()) {
          break;
        }

        trigger = pending_trigger_;
        has_pending_trigger_ = false;
      }

      run_demo_flow(trigger);

      if (!rclcpp::ok()) {
        break;
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (stop_requested_) {
          break;
        }
        state_ = DemoState::Cooldown;
      }

      RCLCPP_INFO(this->get_logger(), "Demo flow finished, enter cooldown.");
      wait_or_stop(kCooldownDuration);

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (stop_requested_) {
          break;
        }
        state_ = DemoState::Idle;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Cooldown finished. Slide left or right to trigger again.");
    }
  }

  void run_demo_flow(uint8_t trigger) {
    RCLCPP_INFO(this->get_logger(), "Start demo flow from trigger: %s",
                trigger_name_from_event(trigger));

    if (!play_emotion(kEmotionId, kEmotionLoopCount)) {
      RCLCPP_WARN(this->get_logger(),
                  "Emotion playback was not accepted. Continue demo flow.");
    }

    if (!set_led_mode(
            aimdk_msgs::srv::LedStripCommand::Request::LED_GREEN_FLOW)) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to set busy light, continue demo flow.");
    }

    play_tts(kOpeningText, true);

    bool critical_step_ok = should_continue();
    if (critical_step_ok) {
      critical_step_ok = set_preset_motion(
          aimdk_msgs::msg::McPresetMotion::BOTH_HANDS_MAKE_HEART, true);
    }

    if (critical_step_ok && should_continue()) {
      wait_or_stop(kPresetMotionDisplayDuration);
    }

    if (critical_step_ok && should_continue()) {
      play_tts(kClosingText, true);
    }

    if (!is_stop_requested() &&
        !set_led_mode(
            aimdk_msgs::srv::LedStripCommand::Request::LED_WHITE_ON)) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to restore idle light after demo flow.");
    }
  }

  bool wait_for_required_services() {
    return wait_for_service(led_strip_client_, kLedStripCommandService) &&
           wait_for_service(play_tts_client_, kPlayTtsService) &&
           wait_for_service(set_preset_motion_client_, kSetPresetMotionService);
  }

  template <typename ClientT>
  bool wait_for_service(const std::shared_ptr<ClientT> &client,
                        const char *service_name) {
    while (rclcpp::ok() && !is_stop_requested()) {
      if (client->wait_for_service(kServiceWaitInterval)) {
        RCLCPP_INFO(this->get_logger(), "Service ready: %s", service_name);
        return true;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service: %s",
                  service_name);
    }
    return false;
  }

  bool set_led_mode(uint8_t led_strip_mode) {
    auto request =
        std::make_shared<aimdk_msgs::srv::LedStripCommand::Request>();
    request->request = aimdk_msgs::msg::CommonRequest();
    request->request.header.stamp = this->now();
    request->led_strip_mode = led_strip_mode;
    request->r = 0;
    request->g = 0;
    request->b = 0;
    request->period = 0;

    auto future = led_strip_client_->async_send_request(request);
    if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(),
                   "LedStripCommand timed out after %ld ms.",
                   kServiceCallTimeout.count());
      return false;
    }

    const auto response = future.get();
    const auto code = response->header.code;
    const auto status_code = response->status_code;
    const bool ok = code == 0 && status_code == 1;

    RCLCPP_INFO(this->get_logger(),
                "LedStripCommand response: code=%ld, status_code=%u", code,
                static_cast<unsigned int>(status_code));

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(),
                   "LedStripCommand rejected for led_strip_mode=%u.",
                   static_cast<unsigned int>(led_strip_mode));
    }
    return ok;
  }

  bool play_tts(const std::string &text, bool wait_for_estimated_duration) {
    auto request = std::make_shared<aimdk_msgs::srv::PlayTts::Request>();
    request->header = aimdk_msgs::msg::CommonRequest();
    request->header.header.stamp = this->now();
    request->tts_req.text = text;
    request->tts_req.domain = kTtsDomain;
    request->tts_req.trace_id = kTtsTraceId;
    request->tts_req.is_interrupted = true;
    request->tts_req.priority_weight = 0;
    request->tts_req.priority_level.value =
        aimdk_msgs::msg::TtsPriorityLevel::INTERACTION_L6;

    auto future = play_tts_client_->async_send_request(request);
    if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "PlayTts timed out after %ld ms.",
                   kServiceCallTimeout.count());
      return false;
    }

    const auto response = future.get();
    const bool ok = response->tts_resp.is_success;
    RCLCPP_INFO(this->get_logger(),
                "PlayTts response: code=%ld, status=%d, is_success=%d, "
                "estimated_duration=%u, message=%s",
                response->header.header.code, response->header.status.value,
                static_cast<int>(response->tts_resp.is_success),
                response->tts_resp.estimated_duration,
                response->tts_resp.error_message.c_str());

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "PlayTts rejected for text: %s",
                   text.c_str());
      return false;
    }

    if (wait_for_estimated_duration) {
      wait_or_stop(std::chrono::milliseconds(response->tts_resp.estimated_duration) +
                   kTtsPlaybackBuffer);
    }

    return true;
  }

  bool play_emotion(int32_t emotion_id, int32_t loop_count) {
    if (!play_emotion_client_->wait_for_service(0s)) {
      RCLCPP_WARN(this->get_logger(), "PlayEmotion service is not ready: %s",
                  kPlayEmotionService);
      return false;
    }

    auto request = std::make_shared<aimdk_msgs::srv::PlayEmotion::Request>();
    request->header = aimdk_msgs::msg::CommonRequest();
    request->header.header.stamp = this->now();
    request->type = "emotion";
    request->priority = 0;
    request->emotion_ids = {emotion_id};
    request->loop_count = loop_count;

    RCLCPP_INFO(this->get_logger(),
                "Sending PlayEmotion request: type=%s, emotion_id=%d, "
                "loop_count=%d",
                request->type.c_str(), emotion_id, loop_count);

    auto future = play_emotion_client_->async_send_request(request);
    if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "PlayEmotion timed out after %ld ms.",
                   kServiceCallTimeout.count());
      return false;
    }

    const auto response = future.get();
    const auto code = response->header.header.code;
    const auto status = response->header.status.value;
    const bool ok =
        code == 0 ||
        status == aimdk_msgs::msg::CommonState::SUCCESS;

    RCLCPP_INFO(this->get_logger(),
                "PlayEmotion response: code=%ld, status=%d, message=%s",
                code, status, response->header.message.c_str());

    if (!ok) {
      RCLCPP_ERROR(this->get_logger(),
                   "PlayEmotion rejected: emotion_id=%d, loop_count=%d",
                   emotion_id, loop_count);
    }

    return ok;
  }

  bool set_preset_motion(int32_t motion_id, bool interrupt) {
    auto request =
        std::make_shared<aimdk_msgs::srv::SetMcPresetMotion::Request>();
    request->header = aimdk_msgs::msg::RequestHeader();
    request->header.stamp = this->now();
    request->motion.value = motion_id;
    request->interrupt = interrupt;
    request->ani_path = "";

    auto future = set_preset_motion_client_->async_send_request(request);
    if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(),
                   "SetMcPresetMotion timed out after %ld ms.",
                   kServiceCallTimeout.count());
      return false;
    }

    const auto response = future.get();
    const auto code = response->response.header.code;
    const auto state = response->response.state.value;

    RCLCPP_INFO(this->get_logger(),
                "SetMcPresetMotion response: motion=%d, code=%ld, state=%d, "
                "task_id=%lu",
                motion_id, code, state, response->response.task_id);

    if (code != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "SetMcPresetMotion rejected with code=%ld.", code);
      return false;
    }

    if (state == aimdk_msgs::msg::CommonState::SUCCESS ||
        state == aimdk_msgs::msg::CommonState::RUNNING) {
      return true;
    }

    RCLCPP_ERROR(this->get_logger(),
                 "SetMcPresetMotion returned unexpected state=%d.", state);
    return false;
  }

  bool wait_or_stop(std::chrono::milliseconds duration) {
    std::unique_lock<std::mutex> lock(state_mutex_);
    return !state_cv_.wait_for(lock, duration,
                               [this] { return stop_requested_; });
  }

  bool is_stop_requested() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return stop_requested_;
  }

  bool should_continue() const {
    return rclcpp::ok() && !is_stop_requested();
  }

  bool is_target_trigger(uint8_t event_type) const {
    return event_type == aimdk_msgs::msg::TouchState::TOUCH_EVENT_CLICK_DOUBLE;
  }

  const char *trigger_name_from_event(uint8_t event_type) const {
    if (event_type ==
        aimdk_msgs::msg::TouchState::TOUCH_EVENT_CLICK_DOUBLE) {
      return "TOUCH_EVENT_CLICK_DOUBLE";
    }
    return "UNKNOWN_TRIGGER";
  }

  const char *state_name(DemoState state) const {
    switch (state) {
    case DemoState::Cooldown:
      return "Cooldown";
    case DemoState::Idle:
      return "Idle";
    case DemoState::Busy:
      return "Busy";
    }
    return "Unknown";
  }

  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  DemoState state_{DemoState::Cooldown};
  bool stop_requested_{false};
  bool has_pending_trigger_{false};
  uint8_t pending_trigger_{aimdk_msgs::msg::TouchState::TOUCH_EVENT_NONE};

  rclcpp::Client<aimdk_msgs::srv::LedStripCommand>::SharedPtr led_strip_client_;
  rclcpp::Client<aimdk_msgs::srv::PlayEmotion>::SharedPtr play_emotion_client_;
  rclcpp::Client<aimdk_msgs::srv::PlayTts>::SharedPtr play_tts_client_;
  rclcpp::Client<aimdk_msgs::srv::SetMcPresetMotion>::SharedPtr
      set_preset_motion_client_;
  rclcpp::Subscription<aimdk_msgs::msg::TouchState>::SharedPtr touch_sub_;

  std::thread worker_thread_;
};

std::shared_ptr<SwipeHeartDemo> g_node = nullptr;

void signal_handler(int signal) {
  if (g_node) {
    g_node->handle_signal(signal);
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

int main(int argc, char *argv[]) {
  try {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    auto node = std::make_shared<SwipeHeartDemo>();
    g_node = node;

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);

    std::thread executor_thread([&executor]() { executor.spin(); });

    int exit_code = 0;
    if (!node->initialize()) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Demo initialization failed.");
        exit_code = 1;
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "Initialization interrupted.");
      }
      node->request_stop();
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }

    if (executor_thread.joinable()) {
      executor_thread.join();
    }

    node->request_stop();
    g_node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return exit_code;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"),
                 "Program exited with exception: %s", e.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
