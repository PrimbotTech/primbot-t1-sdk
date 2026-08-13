/*
 Get MC Motions Example Script

 Description:
   This script demonstrates how to call the GetMcMotions service to retrieve
   the list of all available motions registered in the MC module.
   Each motion includes its name, type, and interrupt information.

 Prerequisites:
   - Robot MC service must be running
   - GetMcMotions service must be available

 Usage:
   ros2 run aimdk_examples_cpp get_mc_motions

 Example:
   ros2 run aimdk_examples_cpp get_mc_motions

 Parameters:
   - None
 */

#include "aimdk_msgs/srv/get_mc_motions.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iomanip>
#include <sstream>
#include <unordered_map>

static const char* motion_type_str(int32_t val) {
  static const std::unordered_map<int32_t, const char*> map = {
      {0, "NONE"},
      {1, "ANIMATION"},
      {2, "MIMIC"},
      {3, "FOUNDATION"},
      {4, "MIMIC_OPENSOURCE"}};
  auto it = map.find(val);
  return it != map.end() ? it->second : "NONE";
}

static const char* intr_type_str(int32_t val) {
  static const std::unordered_map<int32_t, const char*> map = {
      {0, "INTR_NO"}, {1, "INTR_FREE"}, {2, "INTR_BREAKPOINT"}};
  auto it = map.find(val);
  return it != map.end() ? it->second : "INTR_NO";
}

class GetMcMotionsClient : public rclcpp::Node {
public:
  GetMcMotionsClient() : Node("get_mc_motions") {
    client_ = this->create_client<aimdk_msgs::srv::GetMcMotions>(
        "/aimdk_5Fmsgs/srv/GetMcMotions");

    RCLCPP_INFO(this->get_logger(),
                "GetMcMotions client created. Waiting for service: %s",
                "/aimdk_5Fmsgs/srv/GetMcMotions");

    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service not available after 10s. Is MC running?");
      throw std::runtime_error("Service unavailable");
    }

    RCLCPP_INFO(this->get_logger(), "Service is available.");
  }

  bool call() {
    auto request = std::make_shared<aimdk_msgs::srv::GetMcMotions::Request>();
    request->header.stamp.sec = 0;
    request->header.stamp.nanosec = 0;

    RCLCPP_INFO(this->get_logger(), "Sending GetMcMotions request...");

    auto future = client_->async_send_request(request);
    auto status = future.wait_for(std::chrono::seconds(5));

    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(),
                   "GetMcMotions timed out after 5000 ms.");
      return false;
    }

    auto response = future.get();
    if (!response) {
      RCLCPP_ERROR(this->get_logger(),
                   "GetMcMotions returned an empty response.");
      return false;
    }

    // Print results
    int64_t code = response->response.header.code;
    int32_t state = response->response.state.value;

    RCLCPP_INFO(this->get_logger(),
                "Response: code=%ld, state=%d",
                code, state);

    auto& motions = response->motion;
    RCLCPP_INFO(this->get_logger(), "Total motions: %zu", motions.size());

    std::ostringstream oss;
    oss << std::left << std::setfill(' ');
    oss << std::string(70, '-') << "\n";
    oss << std::setw(4) << "#"
        << std::setw(35) << "Name"
        << std::setw(18) << "Type"
        << std::setw(16) << "IntrType"
        << "Breakpoints\n";
    oss << std::string(70, '-') << "\n";

    for (size_t i = 0; i < motions.size(); ++i) {
      auto& m = motions[i];
      oss << std::setw(4) << i
          << std::setw(35) << m.tag
          << std::setw(18) << motion_type_str(m.type.value)
          << std::setw(16) << intr_type_str(m.intr_info.intr_type.value)
          << "[";

      for (size_t j = 0; j < m.intr_info.breakpoint_steps.size(); ++j) {
        if (j > 0) oss << ", ";
        oss << m.intr_info.breakpoint_steps[j];
      }
      oss << "]\n";
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    return true;
  }

private:
  rclcpp::Client<aimdk_msgs::srv::GetMcMotions>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GetMcMotionsClient>();
  bool success = node->call();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
