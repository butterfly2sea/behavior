#pragma once

#include <mutex>
#include <atomic>
#include <unordered_map>
#include <vector>
#include <optional>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/object_location.hpp>
#include <custom_msgs/msg/dis_target.hpp>
#include <custom_msgs/msg/object_attack_designate.hpp>
#include <custom_msgs/msg/multispectral_cam_ctrl.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <mavros_msgs/msg/manual_control.hpp>

#include "behavior_node/data/base_enum.hpp"
#include <log/Logger.hpp>

/**
 * 数据缓存管理类 - 线程安全的数据存储
 */
class Cache {
 private:
  // 原子操作的简单数据
  std::atomic<uint8_t> vehicle_id_{0};
  std::atomic<uint8_t> group_id_{0};
  std::atomic<bool> is_armed_{false};
  std::atomic<FlightMode> flight_mode_{FlightMode::UNKNOWN};
  std::atomic<VehicleType> vehicle_type_{VehicleType::UNKNOWN};
  std::atomic<float> battery_percentage_{0.0f};
  std::atomic<GPSFixType> gps_fix_type_{GPSFixType::NO_GPS};
  std::atomic<bool> offboard_enabled_{false};

  // 需要互斥锁保护的复杂数据
  mutable std::mutex data_mutex_;

  // 飞行器状态数据
  custom_msgs::msg::SimpleVehicle simple_vehicle_;
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Point home_position_;

  // 目标相关数据
  std::vector<custom_msgs::msg::ObjectComputation> detected_objects_;
  std::optional<custom_msgs::msg::ObjectLocation> target_location_;
  std::optional<custom_msgs::msg::ObjectAttackDesignate> attack_designate_;

  // 导航相关数据
  geometry_msgs::msg::Polygon waypoints_;
  custom_msgs::msg::DisTarget current_target_info_;

  // 控制相关数据
  std::optional<sensor_msgs::msg::Joy> joystick_data_;
  std::optional<mavros_msgs::msg::ManualControl> manual_control_;

  // 任务相关数据
  std::optional<custom_msgs::msg::TaskStage> current_task_stage_;
  std::optional<custom_msgs::msg::MultispectralCamCtrl> camera_control_;

  // 其他飞行器信息
  std::unordered_map<uint8_t, custom_msgs::msg::SimpleVehicle> other_vehicles_;

 public:
  Cache() {
    txtLog().info(THISMODULE "Initialized data cache");
  }

  // ============= 基本属性访问 =============

  void setVehicleId(uint8_t id) {
    vehicle_id_.store(id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set vehicle ID: %d", id);
  }

  uint8_t getVehicleId() const {
    return vehicle_id_.load(std::memory_order_relaxed);
  }

  void setGroupId(uint8_t id) {
    group_id_.store(id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set group ID: %d", id);
  }

  uint8_t getGroupId() const {
    return group_id_.load(std::memory_order_relaxed);
  }

  void setArmed(bool armed) {
    is_armed_.store(armed, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set armed state: %s", armed ? "true" : "false");
  }

  bool isArmed() const {
    return is_armed_.load(std::memory_order_relaxed);
  }

  void setFlightMode(FlightMode mode) {
    flight_mode_.store(mode, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set flight mode: %d", static_cast<int>(mode));
  }

  FlightMode getFlightMode() const {
    return flight_mode_.load(std::memory_order_relaxed);
  }

  void setVehicleType(VehicleType type) {
    vehicle_type_.store(type, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set vehicle type: %d", static_cast<int>(type));
  }

  VehicleType getVehicleType() const {
    return vehicle_type_.load(std::memory_order_relaxed);
  }

  void setBatteryPercentage(float percentage) {
    battery_percentage_.store(percentage, std::memory_order_relaxed);
  }

  float getBatteryPercentage() const {
    return battery_percentage_.load(std::memory_order_relaxed);
  }

  void setGPSFixType(GPSFixType fix_type) {
    gps_fix_type_.store(fix_type, std::memory_order_relaxed);
  }

  GPSFixType getGPSFixType() const {
    return gps_fix_type_.load(std::memory_order_relaxed);
  }

  void setOffboardEnabled(bool enabled) {
    offboard_enabled_.store(enabled, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set offboard enabled: %s", enabled ? "true" : "false");
  }

  bool isOffboardEnabled() const {
    return offboard_enabled_.load(std::memory_order_relaxed);
  }

  // ============= 复杂数据访问 =============

  void updateSimpleVehicle(const custom_msgs::msg::SimpleVehicle& vehicle) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    simple_vehicle_ = vehicle;

    // 同时更新相关的原子数据
    setVehicleId(vehicle.id);
    setGroupId(vehicle.grp);
    setArmed(vehicle.lock == 0);
    setFlightMode(static_cast<FlightMode>(vehicle.flymd));
    setVehicleType(static_cast<VehicleType>(vehicle.type));
    setBatteryPercentage(vehicle.bat);
    setGPSFixType(static_cast<GPSFixType>(vehicle.fix));

    // 更新位置信息
    current_position_.x = vehicle.x / 1000.0; // mm to m
    current_position_.y = vehicle.y / 1000.0;
    current_position_.z = vehicle.z / 1000.0;
  }

  custom_msgs::msg::SimpleVehicle getSimpleVehicle() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return simple_vehicle_;
  }

  void setCurrentPosition(const geometry_msgs::msg::Point& position) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_position_ = position;
  }

  geometry_msgs::msg::Point getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_position_;
  }

  void setHomePosition(const geometry_msgs::msg::Point& home) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    home_position_ = home;
    txtLog().info(THISMODULE "Set home position: (%.2f, %.2f, %.2f)",
        home.x, home.y, home.z);
  }

  geometry_msgs::msg::Point getHomePosition() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return home_position_;
  }

  // ============= 目标相关 =============

  void addDetectedObject(const custom_msgs::msg::ObjectComputation& object) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    detected_objects_.push_back(object);

    // 限制列表大小
    if (detected_objects_.size() > 50) {
      detected_objects_.erase(detected_objects_.begin());
    }
  }

  std::vector<custom_msgs::msg::ObjectComputation> getDetectedObjects() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return detected_objects_;
  }

  void clearDetectedObjects() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    detected_objects_.clear();
  }

  void setTargetLocation(const custom_msgs::msg::ObjectLocation& target) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_location_ = target;
  }

  std::optional<custom_msgs::msg::ObjectLocation> getTargetLocation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return target_location_;
  }

  void setAttackDesignate(const custom_msgs::msg::ObjectAttackDesignate& designate) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    attack_designate_ = designate;
  }

  std::optional<custom_msgs::msg::ObjectAttackDesignate> getAttackDesignate() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return attack_designate_;
  }

  // ============= 导航相关 =============

  void setWaypoints(const geometry_msgs::msg::Polygon& waypoints) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    waypoints_ = waypoints;
    txtLog().info(THISMODULE "Set waypoints with %zu points", waypoints.points.size());
  }

  geometry_msgs::msg::Polygon getWaypoints() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return waypoints_;
  }

  void setCurrentTargetInfo(const custom_msgs::msg::DisTarget& target_info) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_target_info_ = target_info;
  }

  custom_msgs::msg::DisTarget getCurrentTargetInfo() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_target_info_;
  }

  // ============= 控制相关 =============

  void setJoystickData(const sensor_msgs::msg::Joy& joy) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    joystick_data_ = joy;
  }

  std::optional<sensor_msgs::msg::Joy> getJoystickData() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return joystick_data_;
  }

  void setManualControl(const mavros_msgs::msg::ManualControl& control) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    manual_control_ = control;
  }

  std::optional<mavros_msgs::msg::ManualControl> getManualControl() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return manual_control_;
  }

  // ============= 任务相关 =============

  void setCurrentTaskStage(const custom_msgs::msg::TaskStage& stage) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_task_stage_ = stage;
  }

  std::optional<custom_msgs::msg::TaskStage> getCurrentTaskStage() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_task_stage_;
  }

  void setCameraControl(const custom_msgs::msg::MultispectralCamCtrl& control) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    camera_control_ = control;
  }

  std::optional<custom_msgs::msg::MultispectralCamCtrl> getCameraControl() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return camera_control_;
  }

  // ============= 其他飞行器 =============

  void updateOtherVehicle(uint8_t id, const custom_msgs::msg::SimpleVehicle& vehicle) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    other_vehicles_[id] = vehicle;
  }

  std::optional<custom_msgs::msg::SimpleVehicle> getOtherVehicle(uint8_t id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = other_vehicles_.find(id);
    if (it != other_vehicles_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  std::vector<custom_msgs::msg::SimpleVehicle> getAllOtherVehicles() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<custom_msgs::msg::SimpleVehicle> result;
    result.reserve(other_vehicles_.size());
    for (const auto& [id, vehicle] : other_vehicles_) {
      result.push_back(vehicle);
    }
    return result;
  }

  void removeOtherVehicle(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    other_vehicles_.erase(id);
  }

  // ============= 实用方法 =============

  bool hasValidGPS() const {
    return getGPSFixType() >= GPSFixType::FIX_3D;
  }

  bool isReadyToFly() const {
    return hasValidGPS() &&
        getBatteryPercentage() > 20.0f &&
        getFlightMode() != FlightMode::UNKNOWN;
  }

  double getDistanceToHome() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    double dx = current_position_.x - home_position_.x;
    double dy = current_position_.y - home_position_.y;
    double dz = current_position_.z - home_position_.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    detected_objects_.clear();
    target_location_.reset();
    attack_designate_.reset();
    waypoints_.points.clear();
    joystick_data_.reset();
    manual_control_.reset();
    current_task_stage_.reset();
    camera_control_.reset();
    other_vehicles_.clear();

    txtLog().info(THISMODULE "Cache cleared");
  }
};
