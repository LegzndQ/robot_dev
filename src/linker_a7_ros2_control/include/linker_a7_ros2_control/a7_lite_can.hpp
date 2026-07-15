#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

struct can_frame;

namespace linker_a7_ros2_control
{

class A7LiteCan
{
public:
  static constexpr std::size_t kJointCount = 7;

  struct JointSample
  {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
    double temperature = 0.0;
    std::chrono::steady_clock::time_point stamp{};
    bool valid = false;
  };

  explicit A7LiteCan(
    std::string interface_name, std::array<std::uint8_t, kJointCount> motor_ids);
  ~A7LiteCan();

  A7LiteCan(const A7LiteCan &) = delete;
  A7LiteCan & operator=(const A7LiteCan &) = delete;

  bool open(std::string & error);
  void close();
  bool is_open() const;

  bool check_motors(std::chrono::milliseconds per_motor_timeout, std::string & error);
  bool start_reporting(std::string & error);
  bool wait_for_reporting(std::chrono::milliseconds timeout, std::string & error);
  bool read_available(std::string & error);

  bool reset_errors(std::string & error);
  bool set_profile_position_mode(std::string & error);
  bool set_velocity_limit(double velocity, std::string & error);
  bool set_acceleration_limit(double acceleration, std::string & error);
  bool enable(std::string & error);
  bool disable(std::string & error);
  bool write_positions(
    const std::array<double, kJointCount> & positions, std::string & error);

  const std::array<JointSample, kJointCount> & samples() const;
  const std::array<std::uint8_t, kJointCount> & motor_ids() const;

  static std::uint32_t make_arbitration_id(std::uint8_t motor_id, std::uint8_t comm_type);
  static std::array<std::uint8_t, 8> encode_register_float(
    std::uint16_t register_id, float value);
  static JointSample decode_report(const std::array<std::uint8_t, 8> & data);

private:
  bool send_frame(
    std::uint8_t motor_id, std::uint8_t comm_type,
    const std::array<std::uint8_t, 8> & data, std::string & error);
  bool write_register_u32(
    std::uint8_t motor_id, std::uint16_t register_id, std::uint32_t value,
    std::string & error);
  bool write_register_float(
    std::uint8_t motor_id, std::uint16_t register_id, float value,
    std::string & error);
  bool read_register(
    std::uint8_t motor_id, std::uint16_t register_id,
    std::chrono::milliseconds timeout, std::array<std::uint8_t, 4> & value,
    std::string & error);
  bool receive_frame(can_frame & frame, int timeout_ms, bool & timed_out, std::string & error);
  void process_report(const can_frame & frame);
  int motor_index(std::uint8_t motor_id) const;

  std::string interface_name_;
  std::array<std::uint8_t, kJointCount> motor_ids_;
  std::array<JointSample, kJointCount> samples_{};
  std::chrono::steady_clock::time_point last_send_time_{};
  int socket_fd_ = -1;
};

}  // namespace linker_a7_ros2_control
