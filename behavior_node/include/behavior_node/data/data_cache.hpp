#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "custom_msgs/msg/simple_vehicle.hpp"
#include "custom_msgs/msg/object_computation.hpp"
#include "custom_msgs/msg/offboard_ctrl.hpp"
#include "base_enum.hpp"

template<typename T>
class ThreadSafeCache {
 private:
  mutable std::shared_mutex mutex_;
  std::shared_ptr<T> data_;
  rclcpp::Time last_update_;

 public:
  void update(std::shared_ptr<T> new_data) {
    std::unique_lock lock(mutex_);
    data_ = std::move(new_data);
    last_update_ = rclcpp::Clock().now();
  }

  std::pair<std::shared_ptr<T>, rclcpp::Time> get() const {
    std::shared_lock lock(mutex_);
    return {data_, last_update_};
  }

  bool isValid(double max_age_sec = 1.0) const {
    std::shared_lock lock(mutex_);
    if (!data_) return false;
    return (rclcpp::Clock().now() - last_update_).seconds() < max_age_sec;
  }
};

class DataCache {
 private:
  ThreadSafeCache<custom_msgs::msg::SimpleVehicle> vehicle_state_;
  ThreadSafeCache<custom_msgs::msg::ObjectComputation> detected_objects_;
  ThreadSafeCache<VehicleType> vehicle_type_;
  ThreadSafeCache<sensor_msgs::msg::Joy> joy_control_;
  ThreadSafeCache<geometry_msgs::msg::Point> home_point_;

 public:
  // 类型安全的访问接口
  auto getVehicleState() const { return vehicle_state_.get().first; }
  auto getVehicleId() const { return isVehicleStateValid() ? getVehicleState()->id : 0; }
  auto getDetectedObjects() const { return detected_objects_.get().first; }
  auto getVehicleType() const { return vehicle_type_.get().first; }
  auto getJoyControl() const { return joy_control_.get().first; }
  auto getHomePoint() const { return home_point_.get().first; }

  // 数据是否有效
  bool isVehicleStateValid() const { return vehicle_state_.isValid(); }
  bool isDetectedObjectsValid() const { return detected_objects_.isValid(); }
  bool isVehicleTypeValid() const { return vehicle_type_.isValid(); }
  bool isJoyControlValid() const { return joy_control_.isValid(); }

  // 更新接口
  void updateVehicleState(custom_msgs::msg::SimpleVehicle::SharedPtr msg) { vehicle_state_.update(std::move(msg)); }
  void updateDetectedObjects(custom_msgs::msg::ObjectComputation::SharedPtr msg) {
    detected_objects_.update(std::move(msg));
  }
  void updateVehicleType(VehicleType type) { vehicle_type_.update(std::make_shared<VehicleType>(type)); }
  void updateJoyControl(sensor_msgs::msg::Joy::SharedPtr msg) { joy_control_.update(std::move(msg)); }
  void updateHomePoint(geometry_msgs::msg::Point::SharedPtr msg) { home_point_.update(std::move(msg)); }
};