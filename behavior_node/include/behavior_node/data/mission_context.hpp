#pragma once

#include <nlohmann/json.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <shared_mutex>
#include <custom_msgs/msg/dis_target.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

// mission_context.hpp
class MissionContext {
 private:
  std::string current_action_; // 当前行为

  std::unordered_map<std::string, nlohmann::json> parameters_; // json参数

  std::unordered_map<std::string, nlohmann::json> triggers_; // json触发条件

  int group_id_; // 组号

  std::vector<uint8_t> group_members_; // 组成员

  geometry_msgs::msg::Polygon offsets_; // 编组偏移量

  geometry_msgs::msg::Polygon way_pts_;//航线点

  geometry_msgs::msg::Point home_point_; // Home位置

  uint8_t target_id_ = 0; // 目标ID

  SetContentType set_type_ = SetContentType::TWO_SWITCH; // 设置类型

  int stage_sn_ = -1; // 当前阶段序号

  std::set<uint8_t> excluded_ids_; // 排除的ID

  custom_msgs::msg::DisTarget current_navigation_info_{};// 当前目标航点信息

  int pre_nav_id_{-1}; // 上一个航点id

  mutable std::shared_mutex mutex_; // 互斥锁

 public:

  // 线程安全的访问方法
  void setAction(const std::string &action) {
    std::unique_lock lock(mutex_);
    current_action_ = action;
  }

  std::string getAction() const {
    std::shared_lock lock(mutex_);
    return current_action_;
  }

  void setParameter(const std::string &key, const nlohmann::json &value) {
    std::unique_lock lock(mutex_);
    parameters_[key] = value;
  }

  nlohmann::json getParameter(const std::string &key) {
    std::shared_lock lock(mutex_);
    return parameters_.count(key) > 0 ? parameters_[key] : nlohmann::json{};
  }

  void setGroup(int group_id) {
    std::unique_lock lock(mutex_);
    group_id_ = group_id;
  }

  int getGroup() const {
    std::shared_lock lock(mutex_);
    return group_id_;
  }

  void setOffsets(const geometry_msgs::msg::Polygon &offsets) {
    std::unique_lock lock(mutex_);
    offsets_ = offsets;
  }

  geometry_msgs::msg::Polygon getOffsets() const {
    std::shared_lock lock(mutex_);
    return offsets_;
  }

  void setWaypoints(const geometry_msgs::msg::Polygon &way_pts) {
    std::unique_lock lock(mutex_);
    way_pts_ = way_pts;
  }

  geometry_msgs::msg::Polygon getWaypoints() const {
    std::shared_lock lock(mutex_);
    return way_pts_;
  }

  void setHomePoint(const geometry_msgs::msg::Point &home_point) {
    std::unique_lock lock(mutex_);
    home_point_ = home_point;
  }

  geometry_msgs::msg::Point getHomePoint() const {
    std::shared_lock lock(mutex_);
    return home_point_;
  }

  void setTargetId(uint8_t id) {
    std::unique_lock lock(mutex_);
    target_id_ = id;
  }

  uint8_t getTargetId() const {
    std::shared_lock lock(mutex_);
    return target_id_;
  }

  void setSetType(SetContentType type) {
    std::unique_lock lock(mutex_);
    set_type_ = type;
  }

  SetContentType getSetType() const {
    std::shared_lock lock(mutex_);
    return set_type_;
  }

  void setStage(int stage) {
    std::unique_lock lock(mutex_);
    stage_sn_ = stage;
  }

  int getStage() const {
    std::shared_lock lock(mutex_);
    return stage_sn_;
  }

  void setGroupId(int group_id) {
    std::unique_lock lock(mutex_);
    group_id_ = group_id;
  }

  int getGroupId() const {
    std::shared_lock lock(mutex_);
    return group_id_;
  }

  void setGroupMembers(const std::vector<uint8_t> &members) {
    std::unique_lock lock(mutex_);
    group_members_ = members;
  }

  std::vector<uint8_t> getGroupMembers() const {
    std::shared_lock lock(mutex_);
    return group_members_;
  }

  void setExcludedIds(const std::set<uint8_t> &ids) {
    std::unique_lock lock(mutex_);
    excluded_ids_ = ids;
  }

  void addExcludedId(uint8_t id) {
    std::unique_lock lock(mutex_);
    excluded_ids_.insert(id);
  }

  void clearExcludedIds() {
    std::unique_lock lock(mutex_);
    excluded_ids_.clear();
  }

  void setTrigger(const std::string &name, const nlohmann::json &value) {
    std::unique_lock lock(mutex_);
    triggers_[name] = value;
  }

  void clear() {
    std::unique_lock lock(mutex_);
    current_action_.clear();
    parameters_.clear();
    triggers_.clear();
    stage_sn_ = -1;
    group_members_.clear();
    excluded_ids_.clear();
  }
};