#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/object_location.hpp>

#include <vector>

/**
 * 设置搜索参数动作节点 - 配置搜索相关参数
 */
class SetSearchParameters : public SyncActionBase<SetSearchParameters> {
 public:
  SetSearchParameters(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<SetSearchParameters>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("areaPointsParam", "areaPoints", "搜索区域参数名"),
        BT::InputPort<std::string>("spdParam", "spd", "搜索速度参数名"),
        BT::InputPort<std::string>("tgtClsParam", "tgtCls", "目标类别参数名"),
        BT::InputPort<std::string>("ptTypParam", "pointTag", "点类型参数名"),
        BT::InputPort<float>("search_altitude", 50.0f, "搜索高度(米)"),
        BT::InputPort<float>("overlap_ratio", 0.3f, "重叠率")
    };
  }

 protected:
  BT::NodeStatus execute() override;
};

/**
 * 模式搜索动作节点 - 按照指定模式进行区域搜索
 */
class PatternSearchAction : public StatefulActionBase<PatternSearchAction> {
 public:
  PatternSearchAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<PatternSearchAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("areaPoints", "搜索区域点"),
        BT::InputPort<std::string>("pattern", "GRID", "搜索模式 (GRID, SPIRAL, ZIGZAG, LINE)"),
        BT::InputPort<float>("spd", 5.0f, "搜索速度(m/s)"),
        BT::InputPort<float>("altitude", 50.0f, "搜索高度(米)"),
        BT::InputPort<TargetClass>("tgtCls", TargetClass::UNKNOWN, "目标类别"),
        BT::InputPort<float>("coverage_width", 50.0f, "覆盖宽度(米)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Polygon search_area_;
  SearchPattern search_pattern_;
  float search_speed_;
  float search_altitude_;
  TargetClass target_class_;
  float coverage_width_;

  std::vector<geometry_msgs::msg::Point> search_waypoints_;
  size_t current_waypoint_index_;
  std::chrono::steady_clock::time_point search_start_time_;

  void generateSearchWaypoints();
  std::vector<geometry_msgs::msg::Point> generateGridPattern();
  std::vector<geometry_msgs::msg::Point> generateSpiralPattern();
  std::vector<geometry_msgs::msg::Point> generateZigzagPattern();
  std::vector<geometry_msgs::msg::Point> generateLinePattern();

  bool moveToNextSearchPoint();
  void performSearchAtPoint();
  bool isSearchComplete();
};

/**
 * 沿线搜索动作节点 - 沿着指定航线进行搜索
 */
class SearchViaLineAction : public StatefulActionBase<SearchViaLineAction> {
 public:
  SearchViaLineAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<SearchViaLineAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("areaPoints", "搜索航线点"),
        BT::InputPort<float>("spd", 5.0f, "搜索速度(m/s)"),
        BT::InputPort<TargetClass>("tgtCls", TargetClass::UNKNOWN, "目标类别"),
        BT::InputPort<float>("search_width", 100.0f, "搜索宽度(米)"),
        BT::InputPort<float>("search_height", 50.0f, "搜索高度(米)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Polygon search_line_;
  float search_speed_;
  TargetClass target_class_;
  float search_width_;
  float search_height_;

  size_t current_segment_;
  std::chrono::steady_clock::time_point segment_start_time_;

  bool moveAlongSearchLine();
  void performSearchAlongSegment();
  bool hasMoreSegments();
};

/**
 * 目标跟踪动作节点 - 跟踪已发现的目标
 */
class TargetTrackingAction : public StatefulActionBase<TargetTrackingAction> {
 public:
  TargetTrackingAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<TargetTrackingAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<uint8_t>("target_id", 0, "目标ID"),
        BT::InputPort<float>("track_distance", 30.0f, "跟踪距离(米)"),
        BT::InputPort<float>("track_altitude", 50.0f, "跟踪高度(米)"),
        BT::InputPort<bool>("continuous_track", true, "持续跟踪")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  uint8_t target_id_;
  float track_distance_;
  float track_altitude_;
  bool continuous_track_;

  custom_msgs::msg::ObjectLocation last_target_location_;
  std::chrono::steady_clock::time_point last_update_time_;
  geometry_msgs::msg::Point predicted_position_;

  bool updateTargetLocation();
  geometry_msgs::msg::Point predictTargetPosition();
  geometry_msgs::msg::Point calculateTrackingPosition(const geometry_msgs::msg::Point& target_pos);
  void sendTrackingControl(const geometry_msgs::msg::Point& position);
  bool isTargetVisible();
};

/**
 * 目标确认动作节点 - 确认目标的详细信息
 */
class TargetConfirmationAction : public StatefulActionBase<TargetConfirmationAction> {
 public:
  TargetConfirmationAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<TargetConfirmationAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<uint8_t>("target_id", 0, "待确认目标ID"),
        BT::InputPort<float>("confirmation_distance", 20.0f, "确认距离(米)"),
        BT::InputPort<float>("confirmation_time", 5.0f, "确认时长(秒)"),
        BT::InputPort<int>("confirmation_angles", 3, "确认角度数量")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  uint8_t target_id_;
  float confirmation_distance_;
  float confirmation_time_;
  int confirmation_angles_;

  std::vector<geometry_msgs::msg::Point> confirmation_points_;
  size_t current_confirmation_index_;
  std::chrono::steady_clock::time_point confirmation_start_time_;
  custom_msgs::msg::ObjectLocation target_location_;

  void generateConfirmationPoints();
  bool moveToConfirmationPoint();
  bool performConfirmation();
  bool isTargetConfirmed();
};

/**
 * 区域扫描动作节点 - 详细扫描指定区域
 */
class AreaScanAction : public StatefulActionBase<AreaScanAction> {
 public:
  AreaScanAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<AreaScanAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("center", "扫描中心点"),
        BT::InputPort<float>("radius", 100.0f, "扫描半径(米)"),
        BT::InputPort<float>("scan_altitude", 30.0f, "扫描高度(米)"),
        BT::InputPort<float>("scan_speed", 3.0f, "扫描速度(m/s)"),
        BT::InputPort<int>("scan_layers", 3, "扫描层数")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Point scan_center_;
  float scan_radius_;
  float scan_altitude_;
  float scan_speed_;
  int scan_layers_;

  std::vector<std::vector<geometry_msgs::msg::Point>> layer_waypoints_;
  int current_layer_;
  size_t current_waypoint_;

  void generateScanWaypoints();
  std::vector<geometry_msgs::msg::Point> generateLayerWaypoints(float radius, float altitude);
  bool scanNextPoint();
  bool isLayerComplete();
  bool isScanComplete();
};

/**
 * 智能搜索动作节点 - 基于AI的智能搜索策略
 */
class IntelligentSearchAction : public StatefulActionBase<IntelligentSearchAction> {
 public:
  IntelligentSearchAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<IntelligentSearchAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("search_area", "搜索区域"),
        BT::InputPort<TargetClass>("target_type", TargetClass::UNKNOWN, "目标类型"),
        BT::InputPort<float>("confidence_threshold", 0.8f, "置信度阈值"),
        BT::InputPort<int>("max_search_time", 600, "最大搜索时间(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Polygon search_area_;
  TargetClass target_type_;
  float confidence_threshold_;
  int max_search_time_;

  std::vector<geometry_msgs::msg::Point> priority_points_;
  std::chrono::steady_clock::time_point search_start_time_;

  void calculatePriorityAreas();
  geometry_msgs::msg::Point selectNextSearchPoint();
  void updateSearchStrategy(const std::vector<custom_msgs::msg::ObjectComputation>& detections);
  bool isSearchEffective();
};