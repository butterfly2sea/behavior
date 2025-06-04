#pragma once

#include <mutex>
#include <atomic>

#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include "behavior_node/data/base_enum.hpp"
#include "behavior_node/core/types.hpp"

class MissionContext {
 private:
  // 原子操作的简单数据
  std::atomic<uint8_t> target_id_{0};
  std::atomic<int> stage_sn_{-1};
  std::atomic<int> group_id_{-1};
  std::atomic<bool> mission_active_{false};
  std::atomic<SetContentType> set_type_{SetContentType::TWO_SWITCH};

  // 复杂数据类型使用轻量级mutex保护
  mutable std::mutex data_mutex_;
  std::string current_action_;
  std::string current_tree_name_;
  std::unordered_map<std::string, nlohmann::json> parameters_;
  std::unordered_map<std::string, nlohmann::json> triggers_;
  std::vector<uint8_t> group_members_;
  geometry_msgs::msg::Polygon offsets_;
  geometry_msgs::msg::Polygon way_pts_;
  geometry_msgs::msg::Point home_point_;
  std::set<uint8_t> excluded_ids_;
  behavior_core::SystemState system_state_{behavior_core::SystemState::INITIALIZING};

  rclcpp::Logger logger_;

 public:
  explicit MissionContext(const std::string& logger_name = "MissionContext")
      : logger_(rclcpp::get_logger(logger_name)) {
    txtLog().info(THISMODULE "Initialized mission context");
  }

  // 原子操作接口
  void setTargetId(uint8_t id) {
    target_id_.store(id, std::memory_order_relaxed);
    RCLCPP_DEBUG(logger_, "Set target ID: %d", id);
  }
  uint8_t getTargetId() const {
    return target_id_.load(std::memory_order_relaxed);
  }

  void setStage(int stage) {
    stage_sn_.store(stage, std::memory_order_relaxed);
    RCLCPP_DEBUG(logger_, "Set stage: %d", stage);
  }
  int getStage() const {
    return stage_sn_.load(std::memory_order_relaxed);
  }

  void setGroupId(int group_id) {
    group_id_.store(group_id, std::memory_order_relaxed);
    RCLCPP_DEBUG(logger_, "Set group ID: %d", group_id);
  }
  int getGroupId() const {
    return group_id_.load(std::memory_order_relaxed);
  }

  void setMissionActive(bool active) {
    mission_active_.store(active, std::memory_order_relaxed);
    txtLog().info(THISMODULE "Mission %s", active ? "activated" : "deactivated");
  }
  bool isMissionActive() const {
    return mission_active_.load(std::memory_order_relaxed);
  }

  void setSetType(SetContentType type) {
    set_type_.store(type, std::memory_order_relaxed);
  }
  SetContentType getSetType() const {
    return set_type_.load(std::memory_order_relaxed);
  }

  // 轻量级mutex保护的接口
  void setAction(const std::string& action) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_action_ = action;
    txtLog().info(THISMODULE "Set action: %s", action.c_str());
  }

  std::string getAction() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_action_;
  }

  void setCurrentTreeName(const std::string& tree_name) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_tree_name_ = tree_name;
    txtLog().info(THISMODULE "Set current tree: %s", tree_name.c_str());
  }

  std::string getCurrentTreeName() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_tree_name_;
  }

  // 参数管理
  void setParameter(const std::string& key, const nlohmann::json& value) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    parameters_[key] = value;
    RCLCPP_DEBUG(logger_, "Set parameter: %s", key.c_str());
  }

  nlohmann::json getParameter(const std::string& key) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = parameters_.find(key);
    return it != parameters_.end() ? it->second : nlohmann::json{};
  }

  std::unordered_map<std::string, nlohmann::json> getAllParameters() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return parameters_;
  }

  bool hasParameter(const std::string& key) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return parameters_.find(key) != parameters_.end();
  }

  // 触发器管理
  void setTrigger(const std::string& name, const nlohmann::json& value) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    triggers_[name] = value;
    RCLCPP_DEBUG(logger_, "Set trigger: %s", name.c_str());
  }

  nlohmann::json getTrigger(const std::string& name) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = triggers_.find(name);
    return it != triggers_.end() ? it->second : nlohmann::json{};
  }

  // 组管理
  void setGroupMembers(const std::vector<uint8_t>& members) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    group_members_ = members;
    txtLog().info(THISMODULE "Set group members, count: %zu", members.size());
  }

  std::vector<uint8_t> getGroupMembers() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return group_members_;
  }

  // Home点管理
  void setHomePoint(const geometry_msgs::msg::Point& home_point) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    home_point_ = home_point;
    txtLog().info(THISMODULE "Set home point: (%.6f, %.6f, %.2f)",
                home_point.x, home_point.y, home_point.z);
  }

  geometry_msgs::msg::Point getHomePoint() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return home_point_;
  }

  // 偏移量管理
  void setOffsets(const geometry_msgs::msg::Polygon& offsets) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    offsets_ = offsets;
    RCLCPP_DEBUG(logger_, "Set offsets, count: %zu", offsets.points.size());
  }

  geometry_msgs::msg::Polygon getOffsets() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return offsets_;
  }

  // 航点管理
  void setWaypoints(const geometry_msgs::msg::Polygon& way_pts) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    way_pts_ = way_pts;
    txtLog().info(THISMODULE "Set waypoints, count: %zu", way_pts.points.size());
  }

  geometry_msgs::msg::Polygon getWaypoints() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return way_pts_;
  }

  // 排除ID管理
  void setExcludedIds(const std::set<uint8_t>& ids) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_ = ids;
    RCLCPP_DEBUG(logger_, "Set excluded IDs, count: %zu", ids.size());
  }

  void addExcludedId(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_.insert(id);
    RCLCPP_DEBUG(logger_, "Added excluded ID: %d", id);
  }

  void clearExcludedIds() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_.clear();
    RCLCPP_DEBUG(logger_, "Cleared excluded IDs");
  }

  std::set<uint8_t> getExcludedIds() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return excluded_ids_;
  }

  // 系统状态管理
  void setSystemState(behavior_core::SystemState state) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    system_state_ = state;
    txtLog().info(THISMODULE "System state changed to: %d", static_cast<int>(state));
  }

  behavior_core::SystemState getSystemState() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return system_state_;
  }

  // 清理操作
  void clear() {
    // 重置原子变量
    target_id_.store(0, std::memory_order_relaxed);
    stage_sn_.store(-1, std::memory_order_relaxed);
    group_id_.store(-1, std::memory_order_relaxed);
    mission_active_.store(false, std::memory_order_relaxed);
    set_type_.store(SetContentType::TWO_SWITCH, std::memory_order_relaxed);

    // 清理复杂数据
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_action_.clear();
    current_tree_name_.clear();
    parameters_.clear();
    triggers_.clear();
    group_members_.clear();
    excluded_ids_.clear();

    txtLog().info(THISMODULE "Cleared mission context");
  }
};