#include <array>
#include <cstdint>

#include "gtest/gtest.h"
#include "linker_a7_ros2_control/a7_lite_can.hpp"

namespace linker_a7_ros2_control
{

TEST(A7LiteProtocol, BuildsExtendedIdentifierPayload)
{
  EXPECT_EQ(A7LiteCan::make_arbitration_id(51, 0x12), 0x1200FD33U);
}

TEST(A7LiteProtocol, EncodesLittleEndianFloatRegisterWrite)
{
  const auto data = A7LiteCan::encode_register_float(0x7016, 1.0F);
  const std::array<std::uint8_t, 8> expected{0x16, 0x70, 0, 0, 0, 0, 0x80, 0x3F};
  EXPECT_EQ(data, expected);
}

TEST(A7LiteProtocol, DecodesMotorReportRanges)
{
  const auto low = A7LiteCan::decode_report({0, 0, 0, 0, 0, 0, 0, 0});
  EXPECT_NEAR(low.position, -12.57, 1e-6);
  EXPECT_NEAR(low.velocity, -33.0, 1e-6);
  EXPECT_NEAR(low.effort, -14.0, 1e-6);
  EXPECT_DOUBLE_EQ(low.temperature, 0.0);

  const auto high = A7LiteCan::decode_report(
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
  EXPECT_NEAR(high.position, 12.57, 1e-6);
  EXPECT_NEAR(high.velocity, 33.0, 1e-6);
  EXPECT_NEAR(high.effort, 14.0, 1e-6);
  EXPECT_DOUBLE_EQ(high.temperature, 6553.5);
}

}  // namespace linker_a7_ros2_control
