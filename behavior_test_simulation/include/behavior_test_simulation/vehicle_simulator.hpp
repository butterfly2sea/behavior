#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include <random>

class VehicleSimulator {
 public:
  struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = -50.0;  // 初始高度50米
    double yaw = 0.0;
  };

  struct TargetPosition {
    double x = 0.0;
    double y = 0.0;
    double z = -50.0;
    double yaw = 0.0;
    bool valid = false;
  };

 private:
  Position current_pos_;
  TargetPosition target_pos_;

  // 飞机状态
  uint8_t vehicle_id_ = 1;
  uint8_t lock_state_ = 0;  // 0: LOCK, 1: UNLOCK
  uint8_t flight_mode_ = 0; // 当前飞行模式

  // 仿真参数
  double max_velocity_ = 5.0;  // 最大速度 m/s
  double position_tolerance_ = 0.5;  // 位置容差

  // 随机数生成器
  std::mt19937 rng_;
  std::uniform_real_distribution<double> noise_dist_;

 public:
  VehicleSimulator(uint8_t vehicle_id = 1);

  // 更新仿真状态
  void update(double dt);

  // 设置目标位置
  void setTarget(const custom_msgs::msg::OffboardCtrl& target);

  // 获取当前状态
  custom_msgs::msg::SimpleVehicle getVehicleState();

  // 控制方法
  void setLockState(bool unlocked) { lock_state_ = unlocked ? 1 : 0; }
  void setFlightMode(uint8_t mode) { flight_mode_ = mode; }
  void setPosition(double x, double y, double z, double yaw = 0.0);

  // 状态查询
  bool isLocked() const { return lock_state_ == 0; }
  uint8_t getFlightMode() const { return flight_mode_; }
  Position getCurrentPosition() const { return current_pos_; }
};