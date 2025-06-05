#pragma once

#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <log/Logger.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include "behavior_node/data/base_enum.hpp"

template<typename T>
class ThreadSafeCache {
 private:
  mutable std::mutex mutex_;
  std::shared_ptr<T> data_;
  rclcpp::Time last_update_;

 public:
  void update(std::shared_ptr<T> new_data) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = std::move(new_data);
    last_update_ = rclcpp::Clock().now();
  }

  std::shared_ptr<T> get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  bool isValid(double max_age_sec = 1.0) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!data_) return false;
    return (rclcpp::Clock().now() - last_update_).seconds() < max_age_sec;
  }

  rclcpp::Time getLastUpdateTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_update_;
  }

  // 是否有数据
  bool hasData() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_ != nullptr;
  }

  // 清除数据
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.reset();
    last_update_ = rclcpp::Time(0);
  }
};

class Cache {
 private:
  ThreadSafeCache<custom_msgs::msg::SimpleVehicle> vehicle_state_;
  ThreadSafeCache<custom_msgs::msg::ObjectComputation> detected_objects_;
  ThreadSafeCache<VehicleType> vehicle_type_;
  ThreadSafeCache<sensor_msgs::msg::Joy> joy_control_;

 public:
  explicit Cache() { txtLog().info(THISMODULE "Initialized data cache"); }

  // 飞机状态相关
  std::shared_ptr<custom_msgs::msg::SimpleVehicle> getVehicleState() const {
    return vehicle_state_.get();
  }

  uint8_t getVehicleId() const {
    auto state = getVehicleState();
    return state ? state->id : 0;
  }

  bool isVehicleStateValid(double max_age_sec = 1.0) const {
    return vehicle_state_.isValid(max_age_sec);
  }

  void updateVehicleState(custom_msgs::msg::SimpleVehicle::SharedPtr msg) {
    if (msg) {
      vehicle_state_.update(std::move(msg));
      txtLog().debug(THISMODULE "Updated vehicle state for ID: %d", msg->id);
    }
  }

  // 检测目标相关
  std::shared_ptr<custom_msgs::msg::ObjectComputation> getDetectedObjects() const {
    return detected_objects_.get();
  }

  bool isDetectedObjectsValid(double max_age_sec = 2.0) const {
    return detected_objects_.isValid(max_age_sec);
  }

  void updateDetectedObjects(custom_msgs::msg::ObjectComputation::SharedPtr msg) {
    if (msg) {
      detected_objects_.update(std::move(msg));
      txtLog().debug(THISMODULE "Updated detected objects, count: %zu", msg->objs.size());
    }
  }

  // 飞机类型相关
  std::shared_ptr<VehicleType> getVehicleType() const {
    return vehicle_type_.get();
  }

  bool isVehicleTypeValid() const {
    return vehicle_type_.isValid(3600.0);
  }

  void updateVehicleType(VehicleType type) {
    vehicle_type_.update(std::make_shared<VehicleType>(type));
    txtLog().info(THISMODULE "Updated vehicle type: %d", static_cast<int>(type));
  }

  // 摇杆控制相关
  std::shared_ptr<sensor_msgs::msg::Joy> getJoyControl() const {
    return joy_control_.get();
  }

  bool isJoyControlValid(double max_age_sec = 0.5) const {
    return joy_control_.isValid(max_age_sec);
  }

  void updateJoyControl(sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg) {
      joy_control_.update(std::move(msg));
    }
  }

  void clearAll() {
    vehicle_state_.clear();
    detected_objects_.clear();
    vehicle_type_.clear();
    joy_control_.clear();
    txtLog().info(THISMODULE "Cleared all cache data");
  }
};