#include <gtest/gtest.h>

#include "agv_ackermann_drive/rear_wheel_speed.hpp"

using agv_ackermann_drive::rearWheelSpeed;

TEST(RearWheelSpeed, BothWheelsGrippingGivesTheCommonSpeed)
{
  EXPECT_DOUBLE_EQ(rearWheelSpeed(2.0, 2.0), 2.0);
}

// 车身翘对角：右后轮悬空空转，左后轮压在地上不动。反馈要看到车没走，力矩才不会被压下去
TEST(RearWheelSpeed, AirborneRightWheelDoesNotHideStalledLeftWheel)
{
  EXPECT_DOUBLE_EQ(rearWheelSpeed(0.0, 3.0), 0.0);
}

TEST(RearWheelSpeed, AirborneLeftWheelDoesNotHideStalledRightWheel)
{
  EXPECT_DOUBLE_EQ(rearWheelSpeed(3.0, 0.0), 0.0);
}

TEST(RearWheelSpeed, ReversingKeepsTheSign)
{
  EXPECT_DOUBLE_EQ(rearWheelSpeed(-1.0, -3.0), -1.0);
}

TEST(RearWheelSpeed, WheelsSpinningOppositeWaysUseTheSlowerOne)
{
  EXPECT_DOUBLE_EQ(rearWheelSpeed(-2.0, 0.5), 0.5);
}
