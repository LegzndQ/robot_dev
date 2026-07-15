#include "linker_a7_ros2_control/a7_lite_can.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace linker_a7_ros2_control
{
namespace
{

constexpr std::uint8_t kMasterId = 0xFD;
constexpr std::uint8_t kCommReport = 0x02;
constexpr std::uint8_t kCommEnable = 0x03;
constexpr std::uint8_t kCommDisable = 0x04;
constexpr std::uint8_t kCommReadRegister = 0x11;
constexpr std::uint8_t kCommWriteRegister = 0x12;
constexpr std::uint8_t kCommActiveReport = 0x18;

constexpr std::uint16_t kRegisterControlMode = 0x7005;
constexpr std::uint16_t kRegisterTargetPosition = 0x7016;
constexpr std::uint16_t kRegisterVelocityLimit = 0x7024;
constexpr std::uint16_t kRegisterAccelerationLimit = 0x7025;
constexpr auto kMinimumSendInterval = std::chrono::microseconds(150);
constexpr auto kTransmitRetryInterval = std::chrono::microseconds(250);
constexpr int kTransmitAttempts = 20;

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

std::uint16_t decode_be_u16(std::uint8_t high, std::uint8_t low)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(high) << 8U) | static_cast<std::uint16_t>(low));
}

std::array<std::uint8_t, 8> encode_register_u32(
  std::uint16_t register_id, std::uint32_t value)
{
  return {
    static_cast<std::uint8_t>(register_id & 0xFFU),
    static_cast<std::uint8_t>((register_id >> 8U) & 0xFFU),
    0,
    0,
    static_cast<std::uint8_t>(value & 0xFFU),
    static_cast<std::uint8_t>((value >> 8U) & 0xFFU),
    static_cast<std::uint8_t>((value >> 16U) & 0xFFU),
    static_cast<std::uint8_t>((value >> 24U) & 0xFFU),
  };
}

}  // namespace

A7LiteCan::A7LiteCan(
  std::string interface_name, std::array<std::uint8_t, kJointCount> motor_ids)
: interface_name_(std::move(interface_name)), motor_ids_(motor_ids)
{
}

A7LiteCan::~A7LiteCan()
{
  close();
}

bool A7LiteCan::open(std::string & error)
{
  close();

  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    error = errno_message("Failed to create SocketCAN socket");
    return false;
  }

  ifreq request{};
  if (interface_name_.size() >= IFNAMSIZ) {
    error = "CAN interface name is too long: " + interface_name_;
    close();
    return false;
  }
  std::strncpy(request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
    error = errno_message("Failed to find CAN interface " + interface_name_);
    close();
    return false;
  }

  const int receive_own_messages = 0;
  if (::setsockopt(
      socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
      &receive_own_messages, sizeof(receive_own_messages)) < 0)
  {
    error = errno_message("Failed to configure SocketCAN receive mode");
    close();
    return false;
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    error = errno_message("Failed to bind SocketCAN interface " + interface_name_);
    close();
    return false;
  }

  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    error = errno_message("Failed to set SocketCAN socket non-blocking");
    close();
    return false;
  }

  return true;
}

void A7LiteCan::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  for (auto & sample : samples_) {
    sample = JointSample{};
  }
  last_send_time_ = {};
}

bool A7LiteCan::is_open() const
{
  return socket_fd_ >= 0;
}

std::uint32_t A7LiteCan::make_arbitration_id(
  std::uint8_t motor_id, std::uint8_t comm_type)
{
  return static_cast<std::uint32_t>(motor_id) |
         (static_cast<std::uint32_t>(kMasterId) << 8U) |
         ((static_cast<std::uint32_t>(comm_type) & 0x1FU) << 24U);
}

std::array<std::uint8_t, 8> A7LiteCan::encode_register_float(
  std::uint16_t register_id, float value)
{
  std::uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "A7 Lite protocol requires 32-bit float");
  std::memcpy(&bits, &value, sizeof(bits));
  return encode_register_u32(register_id, bits);
}

A7LiteCan::JointSample A7LiteCan::decode_report(
  const std::array<std::uint8_t, 8> & data)
{
  const double raw_position = decode_be_u16(data[0], data[1]);
  const double raw_velocity = decode_be_u16(data[2], data[3]);
  const double raw_effort = decode_be_u16(data[4], data[5]);
  const double raw_temperature = decode_be_u16(data[6], data[7]);

  JointSample sample;
  sample.position = raw_position / 65535.0 * 25.14 - 12.57;
  sample.velocity = raw_velocity / 65535.0 * 66.0 - 33.0;
  sample.effort = raw_effort / 65535.0 * 28.0 - 14.0;
  sample.temperature = raw_temperature / 10.0;
  sample.stamp = std::chrono::steady_clock::now();
  sample.valid = true;
  return sample;
}

bool A7LiteCan::send_frame(
  std::uint8_t motor_id, std::uint8_t comm_type,
  const std::array<std::uint8_t, 8> & data, std::string & error)
{
  if (!is_open()) {
    error = "CAN interface is not open";
    return false;
  }

  can_frame frame{};
  frame.can_id = make_arbitration_id(motor_id, comm_type) | CAN_EFF_FLAG;
  frame.can_dlc = static_cast<__u8>(data.size());
  std::copy(data.begin(), data.end(), frame.data);

  if (last_send_time_ != std::chrono::steady_clock::time_point{}) {
    std::this_thread::sleep_until(last_send_time_ + kMinimumSendInterval);
  }

  for (int attempt = 0; attempt < kTransmitAttempts; ++attempt) {
    const ssize_t written = ::write(socket_fd_, &frame, sizeof(frame));
    if (written == static_cast<ssize_t>(sizeof(frame))) {
      last_send_time_ = std::chrono::steady_clock::now();
      return true;
    }
    if (written < 0 &&
      (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS))
    {
      std::this_thread::sleep_for(kTransmitRetryInterval);
      continue;
    }
    error = errno_message("Failed to send A7 Lite CAN frame");
    return false;
  }

  error = "A7 Lite CAN transmit queue remained full after throttled retries";
  return false;
}

bool A7LiteCan::write_register_u32(
  std::uint8_t motor_id, std::uint16_t register_id, std::uint32_t value,
  std::string & error)
{
  return send_frame(
    motor_id, kCommWriteRegister, encode_register_u32(register_id, value), error);
}

bool A7LiteCan::write_register_float(
  std::uint8_t motor_id, std::uint16_t register_id, float value,
  std::string & error)
{
  return send_frame(
    motor_id, kCommWriteRegister, encode_register_float(register_id, value), error);
}

bool A7LiteCan::receive_frame(
  can_frame & frame, int timeout_ms, bool & timed_out, std::string & error)
{
  timed_out = false;
  pollfd descriptor{socket_fd_, POLLIN, 0};
  int poll_result = 0;
  do {
    poll_result = ::poll(&descriptor, 1, timeout_ms);
  } while (poll_result < 0 && errno == EINTR);

  if (poll_result == 0) {
    timed_out = true;
    return true;
  }
  if (poll_result < 0) {
    error = errno_message("Failed while polling A7 Lite CAN socket");
    return false;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    error = "A7 Lite CAN socket reported an error";
    return false;
  }

  ssize_t received = 0;
  do {
    received = ::read(socket_fd_, &frame, sizeof(frame));
  } while (received < 0 && errno == EINTR);

  if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    timed_out = true;
    return true;
  }
  if (received != static_cast<ssize_t>(sizeof(frame))) {
    error = received < 0 ? errno_message("Failed to read A7 Lite CAN frame") :
      "Received a truncated A7 Lite CAN frame";
    return false;
  }
  return true;
}

int A7LiteCan::motor_index(std::uint8_t motor_id) const
{
  const auto found = std::find(motor_ids_.begin(), motor_ids_.end(), motor_id);
  if (found == motor_ids_.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(motor_ids_.begin(), found));
}

void A7LiteCan::process_report(const can_frame & frame)
{
  if ((frame.can_id & CAN_EFF_FLAG) == 0 || frame.can_dlc != 8) {
    return;
  }

  const std::uint32_t arbitration_id = frame.can_id & CAN_EFF_MASK;
  const std::uint8_t comm_type = static_cast<std::uint8_t>((arbitration_id >> 24U) & 0x1FU);
  if (comm_type != kCommReport && comm_type != kCommActiveReport) {
    return;
  }

  const std::uint8_t motor_id = static_cast<std::uint8_t>((arbitration_id >> 8U) & 0xFFU);
  const int index = motor_index(motor_id);
  if (index < 0) {
    return;
  }

  std::array<std::uint8_t, 8> data{};
  std::copy_n(frame.data, data.size(), data.begin());
  samples_[static_cast<std::size_t>(index)] = decode_report(data);
}

bool A7LiteCan::read_register(
  std::uint8_t motor_id, std::uint16_t register_id,
  std::chrono::milliseconds timeout, std::array<std::uint8_t, 4> & value,
  std::string & error)
{
  std::array<std::uint8_t, 8> request{};
  request[0] = static_cast<std::uint8_t>(register_id & 0xFFU);
  request[1] = static_cast<std::uint8_t>((register_id >> 8U) & 0xFFU);
  if (!send_frame(motor_id, kCommReadRegister, request, error)) {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    can_frame frame{};
    bool timed_out = false;
    if (!receive_frame(frame, std::max(1, static_cast<int>(remaining.count())), timed_out, error)) {
      return false;
    }
    if (timed_out) {
      break;
    }

    process_report(frame);
    if ((frame.can_id & CAN_EFF_FLAG) == 0 || frame.can_dlc != 8) {
      continue;
    }
    const std::uint32_t arbitration_id = frame.can_id & CAN_EFF_MASK;
    const std::uint8_t comm_type =
      static_cast<std::uint8_t>((arbitration_id >> 24U) & 0x1FU);
    const std::uint8_t response_motor =
      static_cast<std::uint8_t>((arbitration_id >> 8U) & 0xFFU);
    const std::uint8_t status = static_cast<std::uint8_t>((arbitration_id >> 16U) & 0xFFU);
    if (comm_type == kCommReadRegister && response_motor == motor_id && status == 0) {
      std::copy_n(frame.data + 4, value.size(), value.begin());
      return true;
    }
  }

  std::ostringstream stream;
  stream << "Motor " << static_cast<int>(motor_id) << " did not answer register 0x" <<
    std::hex << std::uppercase << register_id;
  error = stream.str();
  return false;
}

bool A7LiteCan::check_motors(
  std::chrono::milliseconds per_motor_timeout, std::string & error)
{
  std::array<std::uint8_t, kJointCount> missing{};
  std::size_t missing_count = 0;
  for (const auto motor_id : motor_ids_) {
    std::array<std::uint8_t, 4> value{};
    std::string motor_error;
    if (!read_register(
        motor_id, kRegisterTargetPosition, per_motor_timeout, value, motor_error))
    {
      missing[missing_count++] = motor_id;
    }
  }

  if (missing_count == 0) {
    return true;
  }

  std::ostringstream stream;
  stream << "A7 Lite motors did not respond: [";
  for (std::size_t i = 0; i < missing_count; ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << static_cast<int>(missing[i]);
  }
  stream << "]";
  error = stream.str();
  return false;
}

bool A7LiteCan::start_reporting(std::string & error)
{
  const std::array<std::uint8_t, 8> data{1, 2, 3, 4, 5, 6, 1, 0};
  for (const auto motor_id : motor_ids_) {
    if (!send_frame(motor_id, kCommActiveReport, data, error)) {
      return false;
    }
  }
  return true;
}

bool A7LiteCan::wait_for_reporting(
  std::chrono::milliseconds timeout, std::string & error)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::all_of(samples_.begin(), samples_.end(), [](const JointSample & sample) {
        return sample.valid;
      }))
    {
      return true;
    }

    can_frame frame{};
    bool timed_out = false;
    if (!receive_frame(frame, 10, timed_out, error)) {
      return false;
    }
    if (!timed_out) {
      process_report(frame);
    }
  }

  std::ostringstream stream;
  stream << "Timed out waiting for reports from A7 Lite motors [";
  bool first = true;
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    if (!samples_[i].valid) {
      if (!first) {
        stream << ", ";
      }
      stream << static_cast<int>(motor_ids_[i]);
      first = false;
    }
  }
  stream << "]";
  error = stream.str();
  return false;
}

bool A7LiteCan::read_available(std::string & error)
{
  while (true) {
    can_frame frame{};
    const ssize_t received = ::read(socket_fd_, &frame, sizeof(frame));
    if (received < 0 && errno == EINTR) {
      continue;
    }
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return true;
    }
    if (received < 0) {
      error = errno_message("Failed to read A7 Lite CAN state");
      return false;
    }
    if (received != static_cast<ssize_t>(sizeof(frame))) {
      error = "Received a truncated A7 Lite CAN state frame";
      return false;
    }
    process_report(frame);
  }
}

bool A7LiteCan::reset_errors(std::string & error)
{
  const std::array<std::uint8_t, 8> data{1, 0, 0, 0, 0, 0, 0, 0};
  for (const auto motor_id : motor_ids_) {
    if (!send_frame(motor_id, kCommDisable, data, error)) {
      return false;
    }
  }
  return true;
}

bool A7LiteCan::set_profile_position_mode(std::string & error)
{
  for (const auto motor_id : motor_ids_) {
    if (!write_register_u32(motor_id, kRegisterControlMode, 1U, error)) {
      return false;
    }
  }
  return true;
}

bool A7LiteCan::set_velocity_limit(double velocity, std::string & error)
{
  for (const auto motor_id : motor_ids_) {
    if (!write_register_float(
        motor_id, kRegisterVelocityLimit, static_cast<float>(velocity), error))
    {
      return false;
    }
  }
  return true;
}

bool A7LiteCan::set_acceleration_limit(double acceleration, std::string & error)
{
  for (const auto motor_id : motor_ids_) {
    if (!write_register_float(
        motor_id, kRegisterAccelerationLimit, static_cast<float>(acceleration), error))
    {
      return false;
    }
  }
  return true;
}

bool A7LiteCan::enable(std::string & error)
{
  const std::array<std::uint8_t, 8> data{};
  for (const auto motor_id : motor_ids_) {
    if (!send_frame(motor_id, kCommEnable, data, error)) {
      return false;
    }
  }
  return true;
}

bool A7LiteCan::disable(std::string & error)
{
  const std::array<std::uint8_t, 8> data{};
  bool success = true;
  for (const auto motor_id : motor_ids_) {
    if (!send_frame(motor_id, kCommDisable, data, error)) {
      success = false;
    }
  }
  return success;
}

bool A7LiteCan::write_positions(
  const std::array<double, kJointCount> & positions, std::string & error)
{
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (!std::isfinite(positions[i])) {
      error = "Refusing to send a non-finite A7 Lite joint command";
      return false;
    }
    if (!write_register_float(
        motor_ids_[i], kRegisterTargetPosition, static_cast<float>(positions[i]), error))
    {
      return false;
    }
  }
  return true;
}

const std::array<A7LiteCan::JointSample, A7LiteCan::kJointCount> & A7LiteCan::samples() const
{
  return samples_;
}

const std::array<std::uint8_t, A7LiteCan::kJointCount> & A7LiteCan::motor_ids() const
{
  return motor_ids_;
}

}  // namespace linker_a7_ros2_control
