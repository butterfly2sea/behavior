#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/uint8_multi_array.hpp>

#include <vector>
#include <future>

/**
 * 设置编队成员动作节点 - 设置编队中的飞机成员
 */
class SetFormationMembers : public SyncActionBase<SetFormationMembers> {
 public:
  SetFormationMembers(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<SetFormationMembers>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<uint8_t>>("member_ids", "编队成员ID列表"),
        BT::InputPort<uint8_t>("leader_id", 0, "编队长机ID (0表示自动选择)")
    };
  }

 protected:
  BT::NodeStatus execute() override;

 private:
  void publishGroupMembers(const std::vector<uint8_t>& members);
  uint8_t selectLeader(const std::vector<uint8_t>& members);
};

/**
 * 编队类型切换动作节点 - 切换编队队形
 */
class FormationSwitch : public StatefulActionBase<FormationSwitch> {
 public:
  FormationSwitch(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FormationSwitch>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("formation_type", "LINE", "编队类型 (LINE, WEDGE, VEE, DIAMOND, BOX, CIRCLE)"),
        BT::InputPort<float>("formation_spacing", 20.0f, "编队间距(米)"),
        BT::InputPort<float>("transition_time", 10.0f, "转换时间(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::future<custom_msgs::srv::CommandInt::Response::SharedPtr> service_future_;
  FormationMode target_formation_;
  float formation_spacing_;
  float transition_time_;
  bool service_called_ = false;
  std::chrono::steady_clock::time_point transition_start_;

  geometry_msgs::msg::Polygon calculateFormationOffsets(
      FormationMode formation, float spacing, size_t member_count);

  geometry_msgs::msg::Polygon calculateLineFormation(float spacing, size_t count);
  geometry_msgs::msg::Polygon calculateWedgeFormation(float spacing, size_t count);
  geometry_msgs::msg::Polygon calculateVeeFormation(float spacing, size_t count);
  geometry_msgs::msg::Polygon calculateDiamondFormation(float spacing, size_t count);
  geometry_msgs::msg::Polygon calculateBoxFormation(float spacing, size_t count);
  geometry_msgs::msg::Polygon calculateCircleFormation(float spacing, size_t count);

  bool isTransitionComplete();
};

/**
 * 编队飞行动作节点 - 执行编队飞行
 */
class FormationFlyAction : public StatefulActionBase<FormationFlyAction> {
 public:
  FormationFlyAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FormationFlyAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("waypoints", "编队航点"),
        BT::InputPort<float>("speed", 10.0f, "编队飞行速度(m/s)"),
        BT::InputPort<geometry_msgs::msg::Polygon>("offsets", "编队偏移"),
        BT::InputPort<uint8_t>("leader_id", 0, "长机ID"),
        BT::InputPort<bool>("maintain_formation", true, "保持编队")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Polygon waypoints_;
  float formation_speed_;
  geometry_msgs::msg::Polygon formation_offsets_;
  uint8_t leader_id_;
  bool maintain_formation_;

  size_t current_waypoint_;
  bool is_leader_;
  geometry_msgs::msg::Point my_formation_offset_;

  void calculateMyFormationPosition();
  geometry_msgs::msg::Point getLeaderPosition();
  geometry_msgs::msg::Point calculateMyTargetPosition();
  void sendFormationControl();
  bool isFormationIntact();
  bool hasReachedWaypoint();
};

/**
 * 编队集结动作节点 - 编队成员集结到指定位置
 */
class FormationAssemble : public StatefulActionBase<FormationAssemble> {
 public:
  FormationAssemble(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FormationAssemble>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("assembly_point", "集结点"),
        BT::InputPort<float>("assembly_radius", 50.0f, "集结半径(米)"),
        BT::InputPort<float>("assembly_altitude", 100.0f, "集结高度(米)"),
        BT::InputPort<float>("timeout", 300.0f, "集结超时时间(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Point assembly_point_;
  float assembly_radius_;
  float assembly_altitude_;
  float timeout_;

  std::chrono::steady_clock::time_point assembly_start_time_;
  geometry_msgs::msg::Point my_assembly_position_;

  void calculateMyAssemblyPosition();
  bool moveToAssemblyPosition();
  bool areAllMembersAssembled();
  bool isAssemblyTimeout();
};

/**
 * 编队解散动作节点 - 解散编队
 */
class FormationDisband : public SyncActionBase<FormationDisband> {
 public:
  FormationDisband(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<FormationDisband>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("immediate", false, "立即解散"),
        BT::InputPort<geometry_msgs::msg::Point>("disband_center", "解散中心点"),
        BT::InputPort<float>("disband_radius", 100.0f, "解散半径(米)")
    };
  }

 protected:
  BT::NodeStatus execute() override;

 private:
  void sendDisbandCommand();
  geometry_msgs::msg::Point calculateDisbandPosition(
      const geometry_msgs::msg::Point& center,
      float radius,
      uint8_t vehicle_id);
};

/**
 * 编队重组动作节点 - 重新组织编队
 */
class FormationReorganize : public StatefulActionBase<FormationReorganize> {
 public:
  FormationReorganize(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FormationReorganize>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<uint8_t>>("new_members", "新的编队成员"),
        BT::InputPort<std::string>("new_formation", "LINE", "新的编队类型"),
        BT::InputPort<uint8_t>("new_leader", 0, "新的长机ID"),
        BT::InputPort<float>("reorganize_time", 15.0f, "重组时间(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::vector<uint8_t> new_members_;
  FormationMode new_formation_;
  uint8_t new_leader_;
  float reorganize_time_;

  std::chrono::steady_clock::time_point reorganize_start_;

  enum class ReorganizePhase {
    DISBAND_OLD,
    ESTABLISH_NEW,
    COMPLETE
  };

  ReorganizePhase current_phase_;

  void executePhase();
  bool isPhaseComplete();
  void moveToNextPhase();
};

/**
 * 编队同步动作节点 - 同步编队成员的状态和行为
 */
class FormationSynchronize : public StatefulActionBase<FormationSynchronize> {
 public:
  FormationSynchronize(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FormationSynchronize>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("sync_action", "同步的动作名称"),
        BT::InputPort<bool>("wait_for_all", true, "等待所有成员"),
        BT::InputPort<float>("sync_timeout", 30.0f, "同步超时时间(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::string sync_action_;
  bool wait_for_all_;
  float sync_timeout_;

  std::chrono::steady_clock::time_point sync_start_time_;
  std::set<uint8_t> synchronized_members_;

  void broadcastSyncSignal();
  void checkMemberStatus();
  bool areAllMembersSynchronized();
  bool isSyncTimeout();
};

/**
 * 编队避障动作节点 - 编队避障控制
 */
class FormationObstacleAvoidance : public StatefulActionBase<FormationObstacleAvoidance> {
 public:
  FormationObstacleAvoidance(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FormationObstacleAvoidance>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("detection_range", 50.0f, "障碍物检测范围(米)"),
        BT::InputPort<bool>("formation_priority", true, "保持编队优先"),
        BT::InputPort<geometry_msgs::msg::Point>("target", "目标点")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  float detection_range_;
  bool formation_priority_;
  geometry_msgs::msg::Point target_;

  std::vector<geometry_msgs::msg::Point> detected_obstacles_;
  bool avoidance_active_;

  void detectFormationObstacles();
  void coordinateAvoidanceManeuver();
  bool isObstacleCleared();
  void resumeFormation();
};