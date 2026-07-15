#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "linker_a7_ros2_control/a7_lite_can.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace linker_a7_ros2_control
{

class A7LiteSafetyNode : public rclcpp::Node
{
public:
  A7LiteSafetyNode()
  : Node("a7_lite_safety")
  {
    interface_name_ = declare_parameter<std::string>("can_interface", "can0");
    const auto side = declare_parameter<std::string>("side", "right");
    const auto configured_ids = declare_parameter<std::vector<std::int64_t>>(
      "motor_ids", std::vector<std::int64_t>{});

    const int first_id = side == "left" ? 61 : 51;
    for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
      motor_ids_[i] = static_cast<std::uint8_t>(first_id + static_cast<int>(i));
    }
    if (!configured_ids.empty()) {
      if (configured_ids.size() != motor_ids_.size()) {
        throw std::runtime_error("motor_ids must contain exactly 7 values");
      }
      for (std::size_t i = 0; i < motor_ids_.size(); ++i) {
        motor_ids_[i] = static_cast<std::uint8_t>(configured_ids[i]);
      }
    }

    using std::placeholders::_1;
    using std::placeholders::_2;
    emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/linker/arm/emergency_stop",
      std::bind(&A7LiteSafetyNode::disable_motors, this, _1, _2));
    disable_service_ = create_service<std_srvs::srv::Trigger>(
      "/linker/arm/disable",
      std::bind(&A7LiteSafetyNode::disable_motors, this, _1, _2));

    RCLCPP_INFO(
      get_logger(), "A7 Lite emergency disable services ready on %s",
      interface_name_.c_str());
  }

private:
  void disable_motors(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    A7LiteCan can(interface_name_, motor_ids_);
    std::string error;
    if (!can.open(error) || !can.disable(error)) {
      response->success = false;
      response->message = "Failed to disable A7 Lite: " + error;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    response->success = true;
    response->message = "A7 Lite motors disabled; restart ros2_control before moving again";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  std::string interface_name_;
  std::array<std::uint8_t, A7LiteCan::kJointCount> motor_ids_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
};

}  // namespace linker_a7_ros2_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<linker_a7_ros2_control::A7LiteSafetyNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("a7_lite_safety"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
