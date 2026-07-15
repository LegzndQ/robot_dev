#pragma once

#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "linker_a7_ros2_control/a7_lite_can.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace linker_a7_ros2_control
{

class A7LiteSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(A7LiteSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kJointCount = A7LiteCan::kJointCount;

  bool parse_hardware_parameters();
  bool copy_samples_to_state(bool require_fresh);
  void close_hardware(bool disable);

  std::unique_ptr<A7LiteCan> can_;
  std::array<double, kJointCount> positions_{};
  std::array<double, kJointCount> velocities_{};
  std::array<double, kJointCount> efforts_{};
  std::array<double, kJointCount> commands_{};
  std::array<double, kJointCount> last_sent_commands_{};
  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  std::array<double, kJointCount> position_signs_{};
  std::array<double, kJointCount> position_offsets_{};
  std::array<std::uint8_t, kJointCount> motor_ids_{};

  std::string interface_name_ = "can0";
  std::string side_ = "right";
  double velocity_limit_ = 5.0;
  double acceleration_limit_ = 20.0;
  double command_epsilon_ = 1e-5;
  std::chrono::milliseconds state_timeout_{250};
  std::chrono::milliseconds keepalive_period_{100};
  std::chrono::steady_clock::time_point last_write_time_{};
  bool active_ = false;
};

}  // namespace linker_a7_ros2_control
