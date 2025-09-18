#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>

#include <vector>

/**
 * 设置航线参数动作节点 - 配置航线相关参数
 */
class SetLineParameters : public SyncActionBase<SetLineParameters> {
 public:
  SetLineParameters(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<SetLineParameters>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("type", 127, "设置类型掩码"),
        BT::InputPort<float>("antiDis", 8.0f, "防撞距离(米)"),
        BT::InputPort<std::string>("vehiTypParam", "vehiType", "载具类型参数名"),
        BT::InputPort<std::string>("spdParam", "spd", "速度参数名"),
        BT::InputPort<std::string>("disParam", "arvDis", "到点距离参数名"),
        BT::InputPort<std::string>("ptTypParam", "pointTag", "点类型参数名"),
        BT:PortsList<std::string>("ptsParam", "wayPoints", "航点参数名"),
        BT::InputPort<std::string>("lpsParam", "loops", "循环次数参数名")
    };
  }

 protected:
  BT::NodeStatus execute() override;

 private:
  void publishVehicleType(VehicleType type);
  void publishSpeed(float speed);
  void publishArrivalDistance(float distance);
  void publishAntiCollisionDistance(float distance);
  void publishPointTag(PointType tag);
  void publishWaypoints(const geometry_msgs::msg::Polygon& waypoints);
  void publishLoops(uint32_t loops);
};

/**
 * 设置编队偏移动作节点 - 设置编队相对偏移
 */
class SetFormationOffset : public SyncActionBase<SetFormationOffset> {
 public:
  SetFormationOffset(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<SetFormationOffset>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("offsetParam", "formOffset", "编队偏移参数名")
    };
  }

 protected:
  BT::NodeStatus execute() override;
};

/**
 * 导航线飞行动作节点 - 按照设定航线飞行
 */
class NavlineAction : public StatefulActionBase<NavlineAction> {
 public:
  NavlineAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<NavlineAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("wayPoints", "航点列表"),
        BT::InputPort<float>("spd", 5.0f, "飞行速度(m/s)"),
        BT::InputPort<uint32_t>("loops", 1, "循环次数"),
        BT::InputPort<float>("arvDis", 5.0f, "到点距离判断(米)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Polygon waypoints_;
  float speed_;
  uint32_t total_loops_;
  float arrival_distance_;

  uint32_t current_loop_;
  size_t current_waypoint_index_;
  std::chrono::steady_clock::time_point waypoint_start_time_;

  bool isAtWaypoint(const geometry_msgs::msg::Point32& waypoint);
  geometry_msgs::msg::Point32 getCurrentWaypoint();
  bool hasNextWaypoint();
  void moveToNextWaypoint();
  void sendOffboardControl(const geometry_msgs::msg::Point32& target);
};

/**
 * 到达目标点动作节点 - 飞往指定目标点
 */
class GoToDestination : public StatefulActionBase<GoToDestination> {
 public:
  GoToDestination(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<GoToDestination>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("target", "目标点"),
        BT::InputPort<float>("speed", 5.0f, "飞行速度(m/s)"),
        BT::InputPort<float>("arrival_distance", 3.0f, "到达判断距离(米)"),
        BT::InputPort<float>("yaw", -999.0f, "目标偏航角(度,-999表示不设置)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Point target_point_;
  float speed_;
  float arrival_distance_;
  float target_yaw_;
  bool use_yaw_;

  double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
  void sendOffboardControl();
};

/**
 * 设置目标点动作节点 - 动态设置导航目标点
 */
class SetDestinationPoint : public SyncActionBase<SetDestinationPoint> {
 public:
  SetDestinationPoint(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<SetDestinationPoint>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<geometry_msgs::msg::Point>("target", "输出目标点"),
        BT::InputPort<int>("step", 1, "步长或索引"),
        BT::InputPort<float>("obsHgh", -30.0f, "观察高度偏移"),
        BT::InputPort<std::string>("ptsParam", "areaPoints", "区域点参数名"),
        BT::InputPort<std::string>("ptTypParam", "pointTag", "点类型参数名")
    };
  }

 protected:
  BT::NodeStatus execute() override;

 private:
  geometry_msgs::msg::Point calculateNextPoint(
      const geometry_msgs::msg::Polygon& area_points,
      int step, float height_offset, PointType point_type);
};

/**
 * 轨迹跟踪动作节点 - 跟踪目标轨迹
 */
class TrajectoryFollowing : public StatefulActionBase<TrajectoryFollowing> {
 public:
  TrajectoryFollowing(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<TrajectoryFollowing>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("trajectory", "轨迹点列表"),
        BT::InputPort<float>("speed", 5.0f, "跟踪速度(m/s)"),
        BT::InputPort<float>("lookahead_distance", 10.0f, "前瞻距离(米)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::vector<geometry_msgs::msg::Point> trajectory_;
  float speed_;
  float lookahead_distance_;
  size_t current_segment_;

  geometry_msgs::msg::Point calculateLookaheadPoint();
  size_t findClosestSegment();
  void sendTrackingControl(const geometry_msgs::msg::Point& lookahead_point);
};

/**
 * 路径规划动作节点 - 规划从当前位置到目标的路径
 */
class PathPlanning : public StatefulActionBase<PathPlanning> {
 public:
  PathPlanning(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<PathPlanning>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("start", "起始点"),
        BT::InputPort<geometry_msgs::msg::Point>("goal", "目标点"),
        BT::InputPort<geometry_msgs::msg::Polygon>("obstacles", "障碍物点列表"),
        BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("path", "规划的路径")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Point start_point_;
  geometry_msgs::msg::Point goal_point_;
  geometry_msgs::msg::Polygon obstacles_;
  std::vector<geometry_msgs::msg::Point> planned_path_;

  bool planning_complete_;

  std::vector<geometry_msgs::msg::Point> planPath(
      const geometry_msgs::msg::Point& start,
      const geometry_msgs::msg::Point& goal,
      const geometry_msgs::msg::Polygon& obstacles);

  bool isCollisionFree(const geometry_msgs::msg::Point& p1,
                       const geometry_msgs::msg::Point& p2,
                       const geometry_msgs::msg::Polygon& obstacles);
};

/**
 * 航点任务动作节点 - 通用航点任务执行器
 */
class WaypointMission : public StatefulActionBase<WaypointMission> {
 public:
  WaypointMission(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<WaypointMission>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("waypoints", "航点列表"),
        BT::InputPort<std::vector<std::string>>("actions", "每个航点的动作列表"),
        BT::InputPort<float>("speed", 5.0f, "飞行速度"),
        BT::InputPort<float>("arrival_distance", 3.0f, "到达距离"),
        BT::InputPort<bool>("auto_continue", true, "自动继续下一航点")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Polygon waypoints_;
  std::vector<std::string> waypoint_actions_;
  float speed_;
  float arrival_distance_;
  bool auto_continue_;

  size_t current_waypoint_index_;
  bool executing_waypoint_action_;
  std::chrono::steady_clock::time_point action_start_time_;

  bool executeWaypointAction(const std::string& action);
  void moveToWaypoint(size_t index);
};