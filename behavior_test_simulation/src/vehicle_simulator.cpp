#include "behavior_test_simulation/vehicle_simulator.hpp"
#include <cmath>
#include <algorithm>

VehicleSimulator::VehicleSimulator(uint8_t vehicle_id)
    : vehicle_id_(vehicle_id), rng_(std::random_device{}()), noise_dist_(-0.1, 0.1) {
  // 初始化位置
  current_pos_ = {0.0, 0.0, -50.0, 0.0};
  target_pos_ = {0.0, 0.0, -50.0, 0.0, false};
}

void VehicleSimulator::update(double dt) {
  if (!target_pos_.valid) {
    return;
  }

  // 计算到目标的距离和方向
  double dx = target_pos_.x - current_pos_.x;
  double dy = target_pos_.y - current_pos_.y;
  double dz = target_pos_.z - current_pos_.z;
  double dyaw = target_pos_.yaw - current_pos_.yaw;

  // 归一化偏航角差值到 [-π, π]
  while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
  while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

  double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

  // 如果已经到达目标，停止移动
  if (distance < position_tolerance_) {
    current_pos_.x = target_pos_.x;
    current_pos_.y = target_pos_.y;
    current_pos_.z = target_pos_.z;
    current_pos_.yaw = target_pos_.yaw;
    target_pos_.valid = false;
    return;
  }

  // 计算速度向量
  double velocity = std::min(max_velocity_, distance / dt);

  if (distance > 0.0) {
    current_pos_.x += (dx / distance) * velocity * dt;
    current_pos_.y += (dy / distance) * velocity * dt;
    current_pos_.z += (dz / distance) * velocity * dt;
  }

  // 更新偏航角
  double yaw_rate = 1.0; // rad/s
  double max_dyaw = yaw_rate * dt;
  if (std::abs(dyaw) > max_dyaw) {
    current_pos_.yaw += (dyaw > 0 ? max_dyaw : -max_dyaw);
  } else {
    current_pos_.yaw = target_pos_.yaw;
  }

  // 添加小量噪声使仿真更真实
  current_pos_.x += noise_dist_(rng_) * dt;
  current_pos_.y += noise_dist_(rng_) * dt;
  current_pos_.z += noise_dist_(rng_) * dt * 0.1; // 高度噪声更小
}

void VehicleSimulator::setTarget(const custom_msgs::msg::OffboardCtrl& target) {
  target_pos_.x = target.x;
  target_pos_.y = target.y;
  target_pos_.z = target.z;
  target_pos_.yaw = target.yaw;
  target_pos_.valid = true;
}

custom_msgs::msg::SimpleVehicle VehicleSimulator::getVehicleState() {
  custom_msgs::msg::SimpleVehicle state;

  state.id = vehicle_id_;
  state.x = static_cast<int32_t>(current_pos_.x * 1000);  // 转换为毫米
  state.y = static_cast<int32_t>(current_pos_.y * 1000);
  state.z = static_cast<int32_t>(current_pos_.z * 1000);
  state.yaw = static_cast<int32_t>(current_pos_.yaw * 1000); // 转换为毫弧度

  state.vx = 0;  // 简化处理，不计算速度
  state.vy = 0;
  state.vz = 0;

  state.lock = lock_state_;
  state.flymd = flight_mode_;

  // GPS模拟 (简单的本地坐标转GPS)
  state.lat = static_cast<int32_t>((39.9042 + current_pos_.x / 111320.0) * 1e7);
  state.lon = static_cast<int32_t>((116.4074 + current_pos_.y / (111320.0 * std::cos(39.9042 * M_PI / 180.0))) * 1e7);
  state.alt = static_cast<int32_t>(-current_pos_.z * 1000); // 海拔高度为正

  return state;
}

void VehicleSimulator::setPosition(double x, double y, double z, double yaw) {
  current_pos_.x = x;
  current_pos_.y = y;
  current_pos_.z = z;
  current_pos_.yaw = yaw;
  target_pos_.valid = false; // 清除目标
}