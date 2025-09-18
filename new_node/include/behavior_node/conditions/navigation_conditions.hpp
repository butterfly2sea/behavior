#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <custom_msgs/msg/dis_target.hpp>

/**
 * 检查到达目标条件节点 - 检查是否到达指定目标点
 */
class CheckArriveDestination : public ConditionBase<CheckArriveDestination> {
 public:
  CheckArriveDestination(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckArriveDestination>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("target", "目标位置"),
        BT::InputPort<float>("arvdis", 5.0f, "到达判断距离(米)"),
        BT::InputPort<bool>("check_altitude", true, "是否检查高度"),
        BT::InputPort<float>("altitude_tolerance", 3.0f, "高度容差(米)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
  double calculateDistance2D(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
};

/**
 * 检查航线完成条件节点 - 检查是否完成航线飞行
 */
class CheckWaypointComplete : public ConditionBase<CheckWaypointComplete> {
 public:
  CheckWaypointComplete(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckWaypointComplete>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("waypoints", "航点列表"),
        BT::InputPort<uint32_t>("loops", 1, "期望循环次数"),
        BT::InputPort<float>("arrival_distance", 5.0f, "到达判断距离(米)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isAtFinalWaypoint();
  bool hasCompletedAllLoops();
};

/**
 * 检查距离条件节点 - 检查距离特定点的距离
 */
class CheckDistanceCondition : public ConditionBase<CheckDistanceCondition> {
 public:
  CheckDistanceCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckDistanceCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("reference_point", "参考点"),
        BT::InputPort<float>("min_distance", 0.0f, "最小距离(米)"),
        BT::InputPort<float>("max_distance", 1000.0f, "最大距离(米)"),
        BT::InputPort<std::string>("distance_type", "3D", "距离类型 (2D, 3D, horizontal, vertical)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  double calculateDistanceByType(const geometry_msgs::msg::Point& p1,
                                 const geometry_msgs::msg::Point& p2,
                                 const std::string& type);
};

/**
 * 检查Home距离条件节点 - 检查距离Home点的距离
 */
class CheckHomeDistanceCondition : public ConditionBase<CheckHomeDistanceCondition> {
 public:
  CheckHomeDistanceCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckHomeDistanceCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("max_distance", 500.0f, "最大允许距离(米)"),
        BT::InputPort<bool>("use_2d_distance", false, "使用2D距离计算")
    };
  }

 protected:
  bool checkCondition() override;
};

/**
 * 检查路径清晰条件节点 - 检查飞行路径是否畅通
 */
class CheckPathClearCondition : public ConditionBase<CheckPathClearCondition> {
 public:
  CheckPathClearCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckPathClearCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("target", "目标点"),
        BT::InputPort<float>("safety_margin", 5.0f, "安全余量(米)"),
        BT::InputPort<int>("check_resolution", 10, "检查分辨率(点数)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool hasObstacleInPath(const geometry_msgs::msg::Point& start,
                         const geometry_msgs::msg::Point& end,
                         float safety_margin,
                         int resolution);
};

/**
 * 检查航点顺序条件节点 - 检查当前航点索引
 */
class CheckWaypointIndexCondition : public ConditionBase<CheckWaypointIndexCondition> {
 public:
  CheckWaypointIndexCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckWaypointIndexCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<uint32_t>("expected_index", 0, "期望的航点索引"),
        BT::InputPort<std::string>("comparison", "equal", "比较方式 (equal, greater, less, greater_equal, less_equal)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool compareIndex(uint32_t current, uint32_t expected, const std::string& comparison);
};

/**
 * 检查导航状态条件节点 - 检查导航系统状态
 */
class CheckNavigationStatusCondition : public ConditionBase<CheckNavigationStatusCondition> {
 public:
  CheckNavigationStatusCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckNavigationStatusCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("expected_status", "active", "期望状态 (active, paused, stopped, error)")
    };
  }

 protected:
  bool checkCondition() override;
};

/**
 * 检查区域内条件节点 - 检查是否在指定区域内
 */
class CheckInAreaCondition : public ConditionBase<CheckInAreaCondition> {
 public:
  CheckInAreaCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckInAreaCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("area_boundary", "区域边界点"),
        BT::InputPort<bool>("check_altitude", false, "检查高度范围"),
        BT::InputPort<float>("min_altitude", 0.0f, "最低高度(米)"),
        BT::InputPort<float>("max_altitude", 1000.0f, "最高高度(米)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isPointInPolygon(const geometry_msgs::msg::Point& point,
                        const geometry_msgs::msg::Polygon& polygon);
};

/**
 * 检查航线偏离条件节点 - 检查是否偏离预定航线
 */
class CheckCourseDeviationCondition : public ConditionBase<CheckCourseDeviationCondition> {
 public:
  CheckCourseDeviationCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckCourseDeviationCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("planned_path", "计划路径"),
        BT::InputPort<float>("max_deviation", 20.0f, "最大允许偏离距离(米)"),
        BT::InputPort<bool>("check_vertical", false, "检查垂直偏离")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  double calculateDeviationFromPath(const geometry_msgs::msg::Point& current_pos,
                                    const geometry_msgs::msg::Polygon& path);
  geometry_msgs::msg::Point findClosestPointOnPath(const geometry_msgs::msg::Point& point,
                                                   const geometry_msgs::msg::Polygon& path);
};

/**
 * 检查航向角条件节点 - 检查飞行器航向角
 */
class CheckHeadingCondition : public ConditionBase<CheckHeadingCondition> {
 public:
  CheckHeadingCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckHeadingCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("target_heading", 0.0f, "目标航向角(度)"),
        BT::InputPort<float>("heading_tolerance", 10.0f, "航向角容差(度)"),
        BT::InputPort<geometry_msgs::msg::Point>("reference_point", "参考点(用于计算朝向)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  float getCurrentHeading();
  float calculateHeadingToPoint(const geometry_msgs::msg::Point& target);
  float normalizeAngle(float angle);
  float angleDifference(float angle1, float angle2);
};

/**
 * 检查导航精度条件节点 - 检查导航精度是否满足要求
 */
class CheckNavigationAccuracyCondition : public ConditionBase<CheckNavigationAccuracyCondition> {
 public:
  CheckNavigationAccuracyCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckNavigationAccuracyCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("max_horizontal_error", 5.0f, "最大水平误差(米)"),
        BT::InputPort<float>("max_vertical_error", 3.0f, "最大垂直误差(米)"),
        BT::InputPort<float>("max_speed_error", 2.0f, "最大速度误差(m/s)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  float getHorizontalPositionError();
  float getVerticalPositionError();
  float getSpeedError();
};

/**
 * 检查地面目标距离条件节点 - 检查与地面目标的距离
 */
class CheckGroundTargetDistanceCondition : public ConditionBase<CheckGroundTargetDistanceCondition> {
 public:
  CheckGroundTargetDistanceCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckGroundTargetDistanceCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<custom_msgs::msg::DisTarget>("target_info", "目标信息"),
        BT::InputPort<float>("min_distance", 10.0f, "最小距离(米)"),
        BT::InputPort<float>("max_distance", 100.0f, "最大距离(米)")
    };
  }

 protected:
  bool checkCondition() override;
};

/**
 * 检查飞行轨迹条件节点 - 检查飞行轨迹的合理性
 */
class CheckTrajectoryCondition : public ConditionBase<CheckTrajectoryCondition> {
 public:
  CheckTrajectoryCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckTrajectoryCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("planned_trajectory", "计划轨迹"),
        BT::InputPort<float>("max_acceleration", 5.0f, "最大加速度(m/s²)"),
        BT::InputPort<float>("max_turn_rate", 30.0f, "最大转弯率(deg/s)"),
        BT::InputPort<float>("trajectory_smoothness", 0.1f, "轨迹平滑度阈值")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isTrajectoryFeasible(const std::vector<geometry_msgs::msg::Point>& trajectory,
                            float max_accel,
                            float max_turn_rate,
                            float smoothness_threshold);
};