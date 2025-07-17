#pragma once

#include <mutex>
#include <atomic>

#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <custom_msgs/msg/status_task.hpp>
#include <custom_msgs/msg/object_location.hpp>

#include "behavior_node/data/base_enum.hpp"
#include "behavior_node/core/types.hpp"

class MissionContext {
 private:
  // 原子操作的简单数据
  std::atomic<uint8_t> search_target_id_{0};
  std::atomic<uint8_t> attack_target_id_{0};
  std::atomic<int> stage_sn_{-1};
  std::atomic<int> group_id_{-1};
  std::atomic<bool> mission_active_{false};
  std::atomic<float> arrival_distance_{-1.0f};
  std::atomic<float> speed_{-1.0f};
  std::atomic<float> anti_dis_{-1.0f};
  std::atomic<int> trace_attack_type_{0}; // 跟踪打击类型 0：跟踪，1：打击
  std::atomic<uint> loop_count_{0}; // 循环次数
  std::atomic<uint> loop_index_{0}; // 当前循环索引
  std::atomic<uint32_t> wp_id_{0xFFFFFFFF}; // 航线当前航点id
  std::atomic<SetContentType> set_type_{SetContentType::TWO_SWITCH};
  // 任务统计
  std::atomic<size_t> total_tasks_executed_{0};
  std::atomic<size_t> successful_tasks_{0};
  std::atomic<size_t> failed_tasks_{0};

  std::atomic<behavior_core::SystemState> system_state_{behavior_core::SystemState::INITIALIZING};

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
  custom_msgs::msg::TaskStage task_stage_;
  custom_msgs::msg::ObjectLocation attack_obj_loc_;

 public:
  explicit MissionContext() { txtLog().info(THISMODULE "Initialized mission context"); }

  // 原子操作接口
  void setSearchTargetId(uint8_t id) {
    search_target_id_.store(id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set target ID: %d", id);
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

  void setStage(int stage) {
    stage_sn_.store(stage, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set stage: %d", stage);
  }
  int getStage() const {
    return stage_sn_.load(std::memory_order_relaxed);
  }

  void setGroupId(int group_id) {
    group_id_.store(group_id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set group ID: %d", group_id);
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

  void setArrivalDistance(float distance) {
    arrival_distance_.store(distance, std::memory_order_relaxed);
    txtLog().info(THISMODULE "Set arrival distance: %.2f", distance);
  }

  float getArrivalDistance() const {
    return arrival_distance_.load(std::memory_order_relaxed);
  }

  void setSpeed(float speed) {
    speed_.store(speed, std::memory_order_relaxed);
    txtLog().info(THISMODULE "Set speed: %.2f", speed);
  }

  float getSpeed() const {
    return speed_.load(std::memory_order_relaxed);
  }

  void setAntiDis(float dis) {
    anti_dis_.store(dis, std::memory_order_relaxed);
    txtLog().info(THISMODULE "Set anti-dis: %.2f", dis);
  }

  float getAntiDis() const {
    return anti_dis_.load(std::memory_order_relaxed);
  }

  void setTraceAttackType(int type) {
    trace_attack_type_.store(type, std::memory_order_relaxed);
    txtLog().info(THISMODULE "Set trace attack type: %d", type);
  }

  int getTraceAttackType() const {
    return trace_attack_type_.load(std::memory_order_relaxed);
  }

  void setLoopCount(int count) {
    loop_count_.store(count, std::memory_order_relaxed);
    txtLog().info(THISMODULE "Set loop count: %d", count);
  }

  uint getLoopCount() const {
    return loop_count_.load(std::memory_order_relaxed);
  }

  void setLoopIndex(int index) {
    loop_index_.store(index, std::memory_order_relaxed);
    txtLog().info(THISMODULE "Set loop index: %d", index);
  }

  // 自增循环索引
  void incLoopIndex() {
    loop_index_.fetch_add(1, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Increment loop index: %d", getLoopIndex());
  }

  uint getLoopIndex() const {
    return loop_index_.load(std::memory_order_relaxed);
  }

  void setWpId(uint32_t id) {
    wp_id_.store(id, std::memory_order_relaxed);
    txtLog().debug(THISMODULE "Set waypoint ID: %d", id);
  }

  uint32_t getWpId() const {
    return wp_id_.load(std::memory_order_relaxed);
  }

  void setSetType(SetContentType type) {
    set_type_.store(type, std::memory_order_relaxed);
  }
  SetContentType getSetType() const {
    return set_type_.load(std::memory_order_relaxed);
  }

  // 轻量级mutex保护的接口
  void setAction(const std::string &action) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_action_ = action;
    txtLog().info(THISMODULE "Set action: %s", action.c_str());
  }

  std::string getAction() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_action_;
  }

  void setCurrentTreeName(const std::string &tree_name) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_tree_name_ = tree_name;
    txtLog().info(THISMODULE "Set current tree: %s", tree_name.c_str());
  }

  std::string getCurrentTreeName() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return current_tree_name_;
  }

  // 参数管理
  void setParameter(const std::string &key, const nlohmann::json &value) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    parameters_[key] = value;
    txtLog().debug(THISMODULE "Set parameter: %s", key.c_str());
  }

  void setParameters(const nlohmann::json &params){
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto parameters = params.get<std::unordered_map<std::string, nlohmann::json>>();
    txtLog().debug(THISMODULE "Set parameters, count: %zu", parameters.size());
    for (auto const& [key, value] : parameters) {
      parameters_[key] = value;
      txtLog().debug(THISMODULE "Set parameter: %s", key.c_str());
    }
  }

  nlohmann::json getParameter(const std::string &key) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (parameters_.contains(key)) {
      return parameters_.at(key);
    } else {
      txtLog().error(THISMODULE "Parameter not found: %s", key.c_str());
      return {};
    }
  }

  std::unordered_map<std::string, nlohmann::json> getAllParameters() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return parameters_;
  }

  bool hasParameter(const std::string &key) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return parameters_.contains(key);
  }

  // 触发器管理
  void setTrigger(const std::string &name, const nlohmann::json &value) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    triggers_[name] = value;
    txtLog().debug(THISMODULE "Set trigger: %s", name.c_str());
  }

  nlohmann::json getTrigger(const std::string &name) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (triggers_.contains(name)) {
      return triggers_.at(name);
    } else {
      txtLog().error(THISMODULE "Trigger not found: %s", name.c_str());
      return {};
    }
  }

  // 组管理
  void setGroupMembers(const std::vector<uint8_t> &members) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    group_members_ = members;
    txtLog().info(THISMODULE "Set group members, count: %zu", members.size());
  }

  std::vector<uint8_t> getGroupMembers() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return group_members_;
  }

  // Home点管理
  void setHomePoint(const geometry_msgs::msg::Point &home_point) {
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
  void setOffsets(const geometry_msgs::msg::Polygon &offsets) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    offsets_ = offsets;
    txtLog().debug(THISMODULE "Set offsets, count: %zu", offsets.points.size());
  }

  geometry_msgs::msg::Polygon getOffsets() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return offsets_;
  }

  // 航点管理
  void setWaypoints(const geometry_msgs::msg::Polygon &way_pts) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    way_pts_ = way_pts;
    txtLog().info(THISMODULE "Set waypoints, count: %zu", way_pts.points.size());
  }

  geometry_msgs::msg::Polygon getWaypoints() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return way_pts_;
  }

  // 排除ID管理
  void setExcludedIds(const std::set<uint8_t> &ids) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_ = ids;
    txtLog().debug(THISMODULE "Set excluded IDs, count: %zu", ids.size());
  }

  void setExcludedIds(const std::vector<uint8_t> &ids) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_.clear();
    for (auto id : ids) {
      excluded_ids_.insert(id);
    }
    txtLog().debug(THISMODULE "Set excluded IDs, count: %zu", ids.size());
  }

  void addExcludedId(uint8_t id) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_.insert(id);
    txtLog().debug(THISMODULE "Added excluded ID: %d", id);
  }

  void clearExcludedIds() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    excluded_ids_.clear();
    txtLog().debug(THISMODULE "Cleared excluded IDs");
  }

  std::set<uint8_t> getExcludedIds() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return excluded_ids_;
  }

  // 任务阶段管理
  void setTaskStage(const custom_msgs::msg::TaskStage &stage) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    task_stage_ = stage;
    txtLog().info(THISMODULE "Set task stage");
  }
  custom_msgs::msg::TaskStage getTaskStage() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return task_stage_;
  }

  // 攻击目标位置管理
  void setAttackObjLoc(const custom_msgs::msg::ObjectLocation &loc) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    attack_obj_loc_ = loc;
    txtLog().info(THISMODULE "Set attack object location");
  }
  custom_msgs::msg::ObjectLocation getAttackObjLoc() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return attack_obj_loc_;
  }

  // ================================ 任务统计管理 ================================

  void incrementTasksExecuted() {
    total_tasks_executed_.fetch_add(1);
  }

  void incrementSuccessfulTasks() {
    successful_tasks_.fetch_add(1);
  }

  void incrementFailedTasks() {
    failed_tasks_.fetch_add(1);
  }

  size_t getTotalTasksExecuted() const {
    return total_tasks_executed_.load();
  }

  size_t getSuccessfulTasks() const {
    return successful_tasks_.load();
  }

  size_t getFailedTasks() const {
    return failed_tasks_.load();
  }

  double getSuccessRate() const {
    size_t total = total_tasks_executed_.load();
    if (total == 0) return 0.0;
    return static_cast<double>(successful_tasks_.load()) / total;
  }

  // ================================ 状态消息生成 ================================

  custom_msgs::msg::StatusTask generateStatusMessage() const {
    custom_msgs::msg::StatusTask status_msg;

    // 基本信息
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      status_msg.stage = stage_sn_.load();
    }

    // 根据系统状态设置任务状态
    switch (system_state_.load()) {
      case behavior_core::SystemState::INITIALIZING:
        status_msg.status = static_cast<int>(behavior_core::TaskStatus::NOT_READY);
        break;
      case behavior_core::SystemState::RUNNING:
        status_msg.status = static_cast<int>(behavior_core::TaskStatus::ONGOING);
        break;
      case behavior_core::SystemState::PAUSED:
        status_msg.status = static_cast<int>(behavior_core::TaskStatus::ONGOING);
        break;
      case behavior_core::SystemState::ERROR:
        status_msg.status = static_cast<int>(behavior_core::TaskStatus::FAILED);
        break;
      case behavior_core::SystemState::SHUTTING_DOWN:
        status_msg.status = static_cast<int>(behavior_core::TaskStatus::COMPLETE);
        break;
      default:
        status_msg.status = static_cast<int>(behavior_core::TaskStatus::NO_START);
        break;
    }

    // 导航信息
    status_msg.dstwaypt = static_cast<int32_t>(wp_id_.load());
    status_msg.diswaypt = 0.0; // 这个值应该由导航系统更新

    return status_msg;
  }

  // 系统状态管理
  void setSystemState(behavior_core::SystemState state) {
    system_state_.store(state, std::memory_order_relaxed);
    txtLog().info(THISMODULE "System state changed to: %d", static_cast<int>(state));
  }

  behavior_core::SystemState getSystemState() const {
    return system_state_.load(std::memory_order_relaxed);
  }

  std::string getSystemStateString() const {
    switch (system_state_.load()) {
      case behavior_core::SystemState::INITIALIZING: return "INITIALIZING";
      case behavior_core::SystemState::RUNNING: return "RUNNING";
      case behavior_core::SystemState::PAUSED: return "PAUSED";
      case behavior_core::SystemState::ERROR: return "ERROR";
      case behavior_core::SystemState::SHUTTING_DOWN: return "SHUTTING_DOWN";
      default: return "UNKNOWN";
    }
  }

  // 清理操作
  void clear() {
    // 重置原子变量
    search_target_id_.store(0, std::memory_order_relaxed);
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