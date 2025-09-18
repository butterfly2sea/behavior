#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"

#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/object_location.hpp>

/**
 * 检查退出搜索条件节点 - 检查是否应该退出搜索
 */
class CheckQuitSearch : public ConditionBase<CheckQuitSearch> {
 public:
  CheckQuitSearch(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckQuitSearch>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("max_search_time", 600, "最大搜索时间(秒)"),
        BT::InputPort<bool>("quit_on_target_found", true, "发现目标时退出"),
        BT::InputPort<TargetClass>("target_class", TargetClass::UNKNOWN, "目标类别"),
        BT::InputPort<float>("confidence_threshold", 0.8f, "置信度阈值")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool hasFoundValidTarget(TargetClass target_class, float confidence_threshold);
  bool isSearchTimeout(int max_time);
  bool hasReceivedStopCommand();
};

/**
 * 检查目标发现条件节点 - 检查是否发现了目标
 */
class CheckTargetDetected : public ConditionBase<CheckTargetDetected> {
 public:
  CheckTargetDetected(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckTargetDetected>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<TargetClass>("target_type", TargetClass::UNKNOWN, "目标类型"),
        BT::InputPort<float>("min_confidence", 0.7f, "最低置信度"),
        BT::InputPort<int>("detection_count_threshold", 3, "检测次数阈值"),
        BT::InputPort<float>("max_age", 10.0f, "最大目标年龄(秒)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool hasRecentDetection(TargetClass target_type, float max_age);
  int getDetectionCount(TargetClass target_type, float time_window);
  float getHighestConfidence(TargetClass target_type);
};

/**
 * 检查目标锁定条件节点 - 检查目标是否已被锁定
 */
class CheckTargetLocked : public ConditionBase<CheckTargetLocked> {
 public:
  CheckTargetLocked(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckTargetLocked>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<uint8_t>("target_id", 0, "目标ID"),
        BT::InputPort<float>("lock_stability_time", 3.0f, "锁定稳定时间(秒)"),
        BT::InputPort<float>("max_position_variance", 5.0f, "最大位置方差(米)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isTargetStablyLocked(uint8_t target_id, float stability_time, float max_variance);
  float calculatePositionVariance(uint8_t target_id, float time_window);
};

/**
 * 检查目标跟踪条件节点 - 检查目标跟踪状态
 */
class CheckTargetTracking : public ConditionBase<CheckTargetTracking> {
 public:
  CheckTargetTracking(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckTargetTracking>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<uint8_t>("target_id", 0, "目标ID"),
        BT::InputPort<float>("max_track_age", 5.0f, "最大跟踪年龄(秒)"),
        BT::InputPort<float>("min_track_quality", 0.6f, "最低跟踪质量")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isTargetBeingTracked(uint8_t target_id);
  float getTrackAge(uint8_t target_id);
  float getTrackQuality(uint8_t target_id);
};

/**
 * 检查搜索区域完成条件节点 - 检查搜索区域是否已完成
 */
class CheckSearchAreaComplete : public ConditionBase<CheckSearchAreaComplete> {
 public:
  CheckSearchAreaComplete(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckSearchAreaComplete>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("search_area", "搜索区域"),
        BT::InputPort<float>("coverage_threshold", 0.9f, "覆盖率阈值"),
        BT::InputPort<float>("search_resolution", 10.0f, "搜索分辨率(米)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  float calculateCoveragePercentage(const geometry_msgs::msg::Polygon& area,
                                    float resolution);
  bool hasSearchedPoint(const geometry_msgs::msg::Point& point, float resolution);
};

/**
 * 检查目标可见性条件节点 - 检查目标是否可见
 */
class CheckTargetVisible : public ConditionBase<CheckTargetVisible> {
 public:
  CheckTargetVisible(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckTargetVisible>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<uint8_t>("target_id", 0, "目标ID"),
        BT::InputPort<float>("max_distance", 200.0f, "最大可见距离(米)"),
        BT::InputPort<float>("min_elevation_angle", -60.0f, "最小俯视角(度)"),
        BT::InputPort<float>("max_elevation_angle", 30.0f, "最大俯视角(度)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isTargetInVisualRange(uint8_t target_id, float max_distance);
  bool isTargetInViewAngle(uint8_t target_id, float min_elev, float max_elev);
  bool hasLineOfSight(uint8_t target_id);
};

/**
 * 检查搜索模式条件节点 - 检查搜索模式状态
 */
class CheckSearchPatternCondition : public ConditionBase<CheckSearchPatternCondition> {
 public:
  CheckSearchPatternCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckSearchPatternCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("expected_pattern", "GRID", "期望搜索模式"),
        BT::InputPort<float>("pattern_progress", 0.0f, "期望进度(0-1)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  SearchPattern getCurrentSearchPattern();
  float getSearchProgress();
};

/**
 * 检查多目标条件节点 - 检查是否发现多个目标
 */
class CheckMultipleTargets : public ConditionBase<CheckMultipleTargets> {
 public:
  CheckMultipleTargets(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckMultipleTargets>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("min_target_count", 2, "最低目标数量"),
        BT::InputPort<TargetClass>("target_type", TargetClass::UNKNOWN, "目标类型"),
        BT::InputPort<float>("min_confidence", 0.7f, "最低置信度"),
        BT::InputPort<float>("max_age", 30.0f, "最大目标年龄(秒)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  int countValidTargets(TargetClass target_type, float min_confidence, float max_age);
};

/**
 * 检查目标优先级条件节点 - 检查目标优先级
 */
class CheckTargetPriority : public ConditionBase<CheckTargetPriority> {
 public:
  CheckTargetPriority(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckTargetPriority>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<uint8_t>("target_id", 0, "目标ID"),
        BT::InputPort<int>("min_priority", 1, "最低优先级"),
        BT::InputPort<bool>("check_threat_level", true, "检查威胁级别")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  int getTargetPriority(uint8_t target_id);
  int calculateThreatLevel(uint8_t target_id);
};

/**
 * 检查搜索效率条件节点 - 检查搜索效率
 */
class CheckSearchEfficiency : public ConditionBase<CheckSearchEfficiency> {
 public:
  CheckSearchEfficiency(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckSearchEfficiency>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("min_efficiency", 0.5f, "最低效率阈值"),
        BT::InputPort<int>("evaluation_window", 60, "评估时间窗口(秒)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  float calculateSearchEfficiency(int time_window);
  float getAreaCoveredPerTime(int time_window);
  int getTargetsFoundPerTime(int time_window);
};

/**
 * 检查传感器状态条件节点 - 检查搜索相关传感器状态
 */
class CheckSensorStatus : public ConditionBase<CheckSensorStatus> {
 public:
  CheckSensorStatus(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckSensorStatus>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("check_camera", true, "检查相机状态"),
        BT::InputPort<bool>("check_gimbal", true, "检查云台状态"),
        BT::InputPort<bool>("check_laser", false, "检查激光测距仪状态"),
        BT::InputPort<bool>("check_radar", false, "检查雷达状态")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isCameraOperational();
  bool isGimbalOperational();
  bool isLaserOperational();
  bool isRadarOperational();
};

/**
 * 检查搜索时间限制条件节点 - 检查搜索时间限制
 */
class CheckSearchTimeLimit : public ConditionBase<CheckSearchTimeLimit> {
 public:
  CheckSearchTimeLimit(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckSearchTimeLimit>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("time_limit", 300, "搜索时间限制(秒)"),
        BT::InputPort<bool>("extend_on_detection", true, "发现目标时延长时间"),
        BT::InputPort<int>("extension_time", 60, "延长时间(秒)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  int getSearchElapsedTime();
  bool hasRecentDetection(int time_window);
  int getRemainingSearchTime(int time_limit, bool extend_on_detection, int extension_time);
};