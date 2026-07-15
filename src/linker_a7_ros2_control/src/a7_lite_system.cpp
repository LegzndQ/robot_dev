#include "linker_a7_ros2_control/a7_lite_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace linker_a7_ros2_control
{
namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("linker_a7_lite_hardware");

std::vector<std::string> split(const std::string & value)
{
  std::vector<std::string> parts;
  std::stringstream stream(value);
  std::string part;
  while (std::getline(stream, part, ',')) {
    const auto first = part.find_first_not_of(" \t");
    const auto last = part.find_last_not_of(" \t");
    parts.push_back(first == std::string::npos ? "" : part.substr(first, last - first + 1));
  }
  return parts;
}

template<typename T, std::size_t N, typename Converter>
bool parse_array(
  const std::string & text, std::array<T, N> & output, Converter converter,
  const std::string & name)
{
  const auto parts = split(text);
  if (parts.size() != N) {
    RCLCPP_ERROR(
      kLogger, "Hardware parameter '%s' needs %zu comma-separated values, got %zu",
      name.c_str(), N, parts.size());
    return false;
  }
  try {
    for (std::size_t i = 0; i < N; ++i) {
      output[i] = converter(parts[i]);
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      kLogger, "Invalid value in hardware parameter '%s': %s",
      name.c_str(), exception.what());
    return false;
  }
  return true;
}

std::string parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & fallback)
{
  const auto found = parameters.find(name);
  return found == parameters.end() ? fallback : found->second;
}

double double_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, double fallback)
{
  return std::stod(parameter_or(parameters, name, std::to_string(fallback)));
}

long long integer_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, long long fallback)
{
  return std::stoll(parameter_or(parameters, name, std::to_string(fallback)));
}

}  // namespace

hardware_interface::CallbackReturn A7LiteSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(kLogger, "A7 Lite requires 7 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' must expose exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto has_state = [&joint](const std::string & name) {
        return std::any_of(
          joint.state_interfaces.begin(), joint.state_interfaces.end(),
          [&name](const hardware_interface::InterfaceInfo & interface) {
            return interface.name == name;
          });
      };
    if (!has_state(hardware_interface::HW_IF_POSITION) ||
      !has_state(hardware_interface::HW_IF_VELOCITY) ||
      !has_state(hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' must expose position, velocity, and effort state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!parse_hardware_parameters()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  positions_.fill(nan);
  velocities_.fill(nan);
  efforts_.fill(nan);
  commands_.fill(nan);
  last_sent_commands_.fill(nan);

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto & command_interface = info_.joints[i].command_interfaces[0];
    lower_limits_[i] = command_interface.min.empty() ?
      -std::numeric_limits<double>::infinity() : std::stod(command_interface.min);
    upper_limits_[i] = command_interface.max.empty() ?
      std::numeric_limits<double>::infinity() : std::stod(command_interface.max);
  }

  RCLCPP_INFO(
    kLogger, "Configured A7 Lite %s arm on %s with motor IDs %d-%d",
    side_.c_str(), interface_name_.c_str(), motor_ids_.front(), motor_ids_.back());
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool A7LiteSystem::parse_hardware_parameters()
{
  try {
    const auto & parameters = info_.hardware_parameters;
    interface_name_ = parameter_or(parameters, "can_interface", "can0");
    side_ = parameter_or(parameters, "side", "right");
    if (side_ != "left" && side_ != "right") {
      RCLCPP_ERROR(kLogger, "Hardware parameter 'side' must be left or right");
      return false;
    }

    const int first_motor_id = side_ == "right" ? 51 : 61;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      motor_ids_[i] = static_cast<std::uint8_t>(first_motor_id + static_cast<int>(i));
      position_signs_[i] = 1.0;
      position_offsets_[i] = 0.0;
    }

    const auto motor_ids_text = parameter_or(parameters, "motor_ids", "");
    if (!motor_ids_text.empty() && !parse_array(
        motor_ids_text, motor_ids_,
        [](const std::string & value) {
          const int id = std::stoi(value);
          if (id < 1 || id > 127) {
            throw std::out_of_range("motor ID must be in [1, 127]");
          }
          return static_cast<std::uint8_t>(id);
        }, "motor_ids"))
    {
      return false;
    }

    const auto signs_text = parameter_or(parameters, "position_signs", "");
    if (!signs_text.empty() && !parse_array(
        signs_text, position_signs_,
        [](const std::string & value) {return std::stod(value);}, "position_signs"))
    {
      return false;
    }
    if (std::any_of(position_signs_.begin(), position_signs_.end(), [](double sign) {
        return std::abs(std::abs(sign) - 1.0) > 1e-9;
      }))
    {
      RCLCPP_ERROR(kLogger, "Every position_signs value must be 1 or -1");
      return false;
    }

    const auto offsets_text = parameter_or(parameters, "position_offsets", "");
    if (!offsets_text.empty() && !parse_array(
        offsets_text, position_offsets_,
        [](const std::string & value) {return std::stod(value);}, "position_offsets"))
    {
      return false;
    }

    velocity_limit_ = double_parameter(parameters, "velocity_limit", 5.0);
    acceleration_limit_ = double_parameter(parameters, "acceleration_limit", 20.0);
    command_epsilon_ = double_parameter(parameters, "command_epsilon", 1e-5);
    state_timeout_ = std::chrono::milliseconds(
      integer_parameter(parameters, "state_timeout_ms", 250));
    keepalive_period_ = std::chrono::milliseconds(
      integer_parameter(parameters, "command_keepalive_ms", 100));

    if (!(velocity_limit_ > 0.0 && velocity_limit_ <= 50.0)) {
      RCLCPP_ERROR(kLogger, "velocity_limit must be in (0, 50]");
      return false;
    }
    if (!(acceleration_limit_ >= 1.0 && acceleration_limit_ <= 50.0)) {
      RCLCPP_ERROR(kLogger, "acceleration_limit must be in [1, 50]");
      return false;
    }
    if (command_epsilon_ < 0.0 || state_timeout_.count() <= 0 ||
      keepalive_period_.count() <= 0)
    {
      RCLCPP_ERROR(kLogger, "Timing and command epsilon parameters must be positive");
      return false;
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(kLogger, "Invalid A7 Lite hardware parameter: %s", exception.what());
    return false;
  }

  return true;
}

std::vector<hardware_interface::StateInterface> A7LiteSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 3);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &efforts_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> A7LiteSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &commands_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn A7LiteSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  can_ = std::make_unique<A7LiteCan>(interface_name_, motor_ids_);
  std::string error;
  if (!can_->open(error) ||
    !can_->check_motors(std::chrono::milliseconds(50), error) ||
    !can_->start_reporting(error) ||
    !can_->wait_for_reporting(std::chrono::milliseconds(1000), error))
  {
    RCLCPP_ERROR(kLogger, "Failed to configure A7 Lite hardware: %s", error.c_str());
    close_hardware(false);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!copy_samples_to_state(false)) {
    close_hardware(false);
    return hardware_interface::CallbackReturn::ERROR;
  }
  commands_ = positions_;
  last_sent_commands_ = positions_;
  RCLCPP_INFO(kLogger, "A7 Lite CAN state reporting is ready");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn A7LiteSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!can_ || !can_->is_open()) {
    RCLCPP_ERROR(kLogger, "Cannot activate A7 Lite: CAN is not configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  commands_ = positions_;
  last_sent_commands_ = positions_;
  std::array<double, kJointCount> raw_positions{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    raw_positions[i] = position_signs_[i] * (positions_[i] - position_offsets_[i]);
  }

  std::string error;
  if (!can_->reset_errors(error) ||
    !can_->set_profile_position_mode(error) ||
    !can_->set_velocity_limit(velocity_limit_, error) ||
    !can_->set_acceleration_limit(acceleration_limit_, error) ||
    !can_->write_positions(raw_positions, error) ||
    !can_->enable(error))
  {
    RCLCPP_ERROR(kLogger, "Failed to activate A7 Lite hardware: %s", error.c_str());
    close_hardware(true);
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!can_->write_positions(raw_positions, error)) {
    RCLCPP_ERROR(kLogger, "Failed to hold A7 Lite activation position: %s", error.c_str());
    close_hardware(true);
    return hardware_interface::CallbackReturn::ERROR;
  }

  last_write_time_ = std::chrono::steady_clock::now();
  active_ = true;
  RCLCPP_INFO(
    kLogger, "A7 Lite enabled in profile-position streaming mode");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn A7LiteSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (can_ && can_->is_open()) {
    std::string error;
    if (!can_->disable(error)) {
      RCLCPP_ERROR(kLogger, "Failed to disable A7 Lite: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(kLogger, "A7 Lite disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn A7LiteSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  close_hardware(false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn A7LiteSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  close_hardware(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn A7LiteSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  close_hardware(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool A7LiteSystem::copy_samples_to_state(bool require_fresh)
{
  if (!can_) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto & samples = can_->samples();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!samples[i].valid) {
      RCLCPP_ERROR(kLogger, "No state received from motor %d", motor_ids_[i]);
      return false;
    }
    if (require_fresh && now - samples[i].stamp > state_timeout_) {
      RCLCPP_ERROR(
        kLogger, "State from motor %d is stale for %lld ms",
        motor_ids_[i],
        static_cast<long long>(
          std::chrono::duration_cast<std::chrono::milliseconds>(now - samples[i].stamp).count()));
      return false;
    }
    positions_[i] = position_signs_[i] * samples[i].position + position_offsets_[i];
    velocities_[i] = position_signs_[i] * samples[i].velocity;
    efforts_[i] = position_signs_[i] * samples[i].effort;
  }
  return true;
}

hardware_interface::return_type A7LiteSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!can_ || !can_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }
  std::string error;
  if (!can_->read_available(error)) {
    RCLCPP_ERROR(kLogger, "A7 Lite state read failed: %s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return copy_samples_to_state(true) ?
    hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type A7LiteSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  bool changed = false;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(commands_[i])) {
      RCLCPP_ERROR(kLogger, "Joint '%s' command is not finite", info_.joints[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    if (commands_[i] < lower_limits_[i] || commands_[i] > upper_limits_[i]) {
      RCLCPP_ERROR(
        kLogger, "Joint '%s' command %.5f is outside [%.5f, %.5f]",
        info_.joints[i].name.c_str(), commands_[i], lower_limits_[i], upper_limits_[i]);
      return hardware_interface::return_type::ERROR;
    }
    changed = changed || !std::isfinite(last_sent_commands_[i]) ||
      std::abs(commands_[i] - last_sent_commands_[i]) > command_epsilon_;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!changed && now - last_write_time_ < keepalive_period_) {
    return hardware_interface::return_type::OK;
  }

  std::array<double, kJointCount> raw_commands{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    raw_commands[i] = position_signs_[i] * (commands_[i] - position_offsets_[i]);
  }

  std::string error;
  if (!can_->write_positions(raw_commands, error)) {
    RCLCPP_ERROR(kLogger, "A7 Lite command write failed: %s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  last_sent_commands_ = commands_;
  last_write_time_ = now;
  return hardware_interface::return_type::OK;
}

void A7LiteSystem::close_hardware(bool disable)
{
  active_ = false;
  if (can_) {
    if (disable && can_->is_open()) {
      std::string error;
      if (!can_->disable(error)) {
        RCLCPP_ERROR(kLogger, "Failed to disable A7 Lite while closing: %s", error.c_str());
      }
    }
    can_->close();
    can_.reset();
  }
}

}  // namespace linker_a7_ros2_control

PLUGINLIB_EXPORT_CLASS(
  linker_a7_ros2_control::A7LiteSystem, hardware_interface::SystemInterface)
