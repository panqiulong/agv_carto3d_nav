#pragma once

#include <cmath>

namespace agv_ackermann_drive
{
// 速度反馈用的后轮转速：取两个后轮里绝对值小的那个。车无悬挂，车身翘对角时一个后轮悬空空转，
// 它的转速不代表车速；只读这个轮子时力矩被压到接近 0，着地的轮子使不上劲
inline double rearWheelSpeed(double left, double right)
{
  return std::abs(left) < std::abs(right) ? left : right;
}
}  // namespace agv_ackermann_drive
