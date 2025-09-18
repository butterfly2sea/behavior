#pragma once

#include <mutex>
#include <atomic>
#include <set>
#include <unordered_map>
#include <string>

#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <custom_msgs/msg/object_location.hpp>

#include "behavior_node/data/base_enum.hpp"
#include "behavior_node/core/types.hpp"
#include <log/Logger.hpp>

/**
 * 任务上下文管理类 - 管理当前任务的状态和参数
 */
class MissionContext {
 private:
  // 原子操作的简单数据
  std::atomic<uint8_t> search_target_id_{0};
  std::atomic<uint8_t> attack_target_id_{0};
  std::atomic<int> group_id_{-1};
  std::atomic<bool> mission_active_{false};
  std::atomic<float> arrival_distance_{5.0f};
  std::atomic<float> speed_{5.0f};
  std::atomic<float> anti_collision_distance_{8.0f};
  std::atomic<int> trace_attack_type_{0}; // 0: 跟踪, 1: 打击
  std::atomic<uint32_t> loop_count_{1}; // 循环次数
  std::atomic<uint32_t> loop_index_{0}; // 当前循环索引
  std::atomic<uint32_t> waypoint_id_{0xFFFFFFFF}; // 当前航点ID
  std::atomic<SetContentType> set_type_{SetContentType::TWO_SWITCH};
  std::atomic<behavior_core::SystemState> system_state_{behavior_core::SystemState::INITIALIZING};
  std::atomic<behavior_core::MissionPhase> mission_phase_{behavior_core::MissionPhase::NONE};

  // 复杂数据类型使用轻量级mutex保护
  mutable std::mutex data_mutex_;

  std::string current_action_;
  std::string current_tree_name_;
  std::string ground_station_command_; // 地面站指令
  std::unordered_map<std::string, nlohmann::json> parameters_;
  std::unordered_map<std::string, nlohmann::json> triggers_;
  std::vector<uint8_t> group_members_;
  geometry_msgs::msg::Polygon formation_offsets_;
  geometry_msgs::msg::Polygon waypoints_;
  geometry_msgs::msg::Point home_point_;
  geometry_msgs::msg::Point target_point_;
  std::set<uint8_t> excluded_ids_;
  custom_msgs::msg::ObjectLocation attack_target_location_;

  // 任务执行状态
  std::chrono::steady_clock::time_point mission_start_time_;
  std::chrono::steady_clock::time_point phase_start_time_;
  ExecutionState execution_state_{ExecutionState::IDLE};

 public:
  explicit MissionContext() {
    txtLog().info(THISMODULE "Initialized mission context");
  }

  // ============= 目标ID管理 =============

  void setSearchTargetId(uint8_t id) {
    search_target_id_.store(id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set search target ID: %d", id);
  }

  uint8_t getSearchTargetId() const {
    return search_target_id_.load(std::memory_order_relaxed);
  }

  void setAttackTargetId(uint8_t id) {
    attack_target_id_.store(id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set attack target ID: %d", id);
  }

  uint8_t getAttackTargetId() const {
    return attack_target_id_.load(std::memory_order_relaxed);
  }

  // ============= 分组管理 =============

  void setGroupId(int group_id) {
    group_id_.store(group_id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set group ID: %d", group_id);
  }

  int getGroupId() const {
    return group_id_.load(std::memory_order_relaxed);
  }

  void setGroupMembers(const std::vector<uint8_t>& members) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    group_members_ = members;
    txtLog().info(THISMODULE "Set group members: %zu members", members.size());
  }

  std::vector<uint8_t> getGroupMembers() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return group_members_;
  }

  // ============= 任务状态管理 =============

  void setMissionActive(bool active) {
    mission_active_.store(active, std::memory_order_relaxed);
    if (active) {
      mission_start_time_ = std::chrono::steady_clock::now();
    }
    txtLog().info(THISMODULE "Mission %s", active ? "activated" : "deactivated");
  }

  bool isMissionActive() const {
    return mission_active_.load(std::memory_order_relaxed);
  }

  void setSystemState(behavior_core::SystemState state) {
    system_state_.store(state, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set system state: %d", static_cast<int>(state));
  }

  behavior_core::SystemState getSystemState() const {
    return system_state_.load(std::memory_order_relaxed);
  }

  void setMissionPhase(behavior_core::MissionPhase phase) {
    mission_phase_.store(phase, std::memory_order_relaxed);
    phase_start_time_ = std::chrono::steady_clock::now();
    txtLog().info(THISMODULE "Set mission phase: %d", static_cast<int>(phase));
  }

  behavior_core::MissionPhase getMissionPhase() const {
    return mission_phase_.load(std::memory_order_relaxed);
  }

  void setExecutionState(ExecutionState state) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    execution_state_ = state;
    txtLog().debug(THISMODULE "Set execution state: %d", static_cast<int>(state));
  }

  ExecutionState getExecutionState() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return execution_state_;
  }

  // ============= 飞行参数管理 =============

  void setArrivalDistance(float distance) {
    arrival_distance_.store(distance, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set arrival distance: %.2f m", distance);
  }

  float getArrivalDistance() const {
    return arrival_distance_.load(std::memory_order_relaxed);
  }

  void setSpeed(float speed) {
    speed_.store(speed, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set speed: %.2f m/s", speed);
  }

  float getSpeed() const {
    return speed_.load(std::memory_order_relaxed);
  }

  void setAntiCollisionDistance(float distance) {
    anti_collision_distance_.store(distance, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set anti-collision distance: %.2f m", distance);
  }

  float getAntiCollisionDistance() const {
    return anti_collision_distance_.load(std::memory_order_relaxed);
  }

  // ============= 跟踪攻击管理 =============

  void setTraceAttackType(int type) {
    trace_attack_type_.store(type, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set trace attack type: %d", type);
  }

  int getTraceAttackType() const {
    return trace_attack_type_.load(std::memory_order_relaxed);
  }

  void setAttackTargetLocation(const custom_msgs::msg::ObjectLocation& location) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    attack_target_location_ = location;
  }

  custom_msgs::msg::ObjectLocation getAttackTargetLocation() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return attack_target_location_;
  }

  // ============= 循环控制 =============

  void setLoopCount(uint32_t count) {
    loop_count_.store(count, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set loop count: %u", count);
  }

  uint32_t getLoopCount() const {
    return loop_count_.load(std::memory_order_relaxed);
  }

  void setLoopIndex(uint32_t index) {
    loop_index_.store(index, std::memory_order_relaxed);
  }

  uint32_t getLoopIndex() const {
    return loop_index_.load(std::memory_order_relaxed);
  }

  void incrementLoopIndex() {
    uint32_t current = loop_index_.load(std::memory_order_relaxed);
    loop_index_.store(current + 1, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Incremented loop index to: %u", current + 1);
  }

  bool isLoopComplete() const {
    uint32_t index = loop_index_.load(std::memory_order_relaxed);
    uint32_t count = loop_count_.load(std::memory_order_relaxed);
    return index >= count;
  }

  // ============= 航点管理 =============

  void setWaypointId(uint32_t id) {
    waypoint_id_.store(id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set waypoint ID: %u", id);
  }

  uint32_t getWaypointId() const {
    return waypoint_id_.load(std::memory_order_relaxed);
  }

  void setWaypoints(const geometry_msgs::msg::Polygon& waypoints) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    waypoints_ = waypoints;
    txtLog().info(THISMODULE "Set waypoints with %zu points", waypoints.points.size());
  }

  geometry_msgs::msg::Polygon getWaypoints() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return waypoints_;
  }

  // ============= 设置类型管理 =============

  void setSetType(SetContentType type) {
    set_type_.store(type, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set type: %d", static_cast<int>(type));
  }

  SetContentType getSetType() const {
    return set_type_.load(std::memory_order_relaxed);
  }

  // ============= 位置管理 =============

  void setHomePoint(const geometry_msgs::msg::Point& home) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    home_point_ = home;
    txtLog().info(THISMODULE "Set home point: (%.2f, %.2f, %.2f)",
        home.x, home.y, home.z);
  }

  geometry_msgs::msg::Point getHomePoint() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return home_point_;
  }

  void setTargetPoint(const geometry_msgs::msg::Point& target) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_point_ = target;
    txtLog().debug(THISMODULE "Set target point: (%.2f, %.2f, %.2f)",
        target.x, target.y, target.z);
  }

  geometry_msgs::msg::Point getTargetPoint() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return target_point_;
  }

  // ============= 编队管理 =============

  void setFormationOffsets(const geometry_msgs::msg::Polygon& offsets) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    formation_offsets_ = offsets;
    txtLog().info(THISMODULE "Set formation offsets with %zu offsets", offsets.points.size());
  }

  geometry_msgs::msg::Polygon getFormationOffsets() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return formation_offsets_;
  }

  // ============= 排除ID管理 =============

  void addExcludedId(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_.insert(id);
    txtLog().debug(THISMODULE "Added excluded ID: %d", id);
  }

  void removeExcludedId(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_.erase(id);
    txtLog().debug(THISMODULE "Removed excluded ID: %d", id);
  }

  std::set<uint8_t> getExcludedIds() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return excluded_ids_;
  }

  bool isExcluded(uint8_t id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return excluded_ids_.find(id) != excluded_ids_.end();
  }

  // ============= 动作和树管理 =============

  void setCurrentAction(const std::string& action) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_action_ = action;
    txtLog().debug(THISMODULE "Set current action: %s", action.c_str());
  }

  std::string getCurrentAction() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_action_;
  }

  void setCurrentTreeName(const std::string& tree_name) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_tree_name_ = tree_name;
    txtLog().debug(THISMODULE "Set current tree name: %s", tree_name.c_str());
  }

  std::string getCurrentTreeName() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_tree_name_;
  }

  // ============= 地面站指令管理 =============

  void setGroundStationCommand(const std::string& command) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ground_station_command_ = command;
    txtLog().info(THISMODULE "Set ground station command: %s", command.c_str());
  }

  std::string getGroundStationCommand() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return ground_station_command_;
  }

  void clearGroundStationCommand() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ground_station_command_.clear();
  }

  // ============= 参数管理 =============

  void setParameter(const std::string& key, const nlohmann::json& value) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    parameters_[key] = value;
    txtLog().debug(THISMODULE "Set parameter: %s", key.c_str());
  }

  std::optional<nlohmann::json> getParameter(const std::string& key) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = parameters_.find(key);
    if (it != parameters_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  void setTrigger(const std::string& key, const nlohmann::json& value) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    triggers_[key] = value;
    txtLog().debug(THISMODULE "Set trigger: %s", key.c_str());
  }

  std::optional<nlohmann::json> getTrigger(const std::string& key) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = triggers_.find(key);
    if (it != triggers_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  // ============= 时间信息 =============

  std::chrono::duration<double> getMissionDuration() const {
    if (!mission_active_.load(std::memory_order_relaxed)) {
      return std::chrono::duration<double>::zero();
    }
    return std::chrono::steady_clock::now() - mission_start_time_;
  }

  std::chrono::duration<double> getPhaseDuration() const {
    return std::chrono::steady_clock::now() - phase_start_time_;
  }

  // ============= 实用方法 =============

  void reset() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 重置原子变量
    search_target_id_.store(0, std::memory_order_relaxed);
    attack_target_id_.store(0, std::memory_order_relaxed);
    group_id_.store(-1, std::memory_order_relaxed);
    mission_active_.store(false, std::memory_order_relaxed);
    arrival_distance_.store(5.0f, std::memory_order_relaxed);
    speed_.store(5.0f, std::memory_order_relaxed);
    anti_collision_distance_.store(8.0f, std::memory_order_relaxed);
    trace_attack_type_.store(0, std::memory_order_relaxed);
    loop_count_.store(1, std::memory_order_relaxed);
    loop_index_.store(0, std::memory_order_relaxed);
    waypoint_id_.store(0xFFFFFFFF, std::memory_order_relaxed);
    set_type_.store(SetContentType::TWO_SWITCH, std::memory_order_relaxed);
    system_state_.store(behavior_core::SystemState::IDLE, std::memory_order_relaxed);
    mission_phase_.store(behavior_core::MissionPhase::NONE, std::memory_order_relaxed);

    // 重置复杂数据
    current_action_.clear();
    current_tree_name_.clear();
    ground_station_command_.clear();
    parameters_.clear();
    triggers_.clear();
    group_members_.clear();
    formation_offsets_.points.clear();
    waypoints_.points.clear();
    excluded_ids_.clear();
    execution_state_ = ExecutionState::IDLE;

    txtLog().info(THISMODULE "Mission context reset");
  }
};