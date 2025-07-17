#pragma once

#include "behavior_node/base_nodes.hpp"

#include <geometry_msgs/msg/point.hpp>

class NavlineAction : public StatefulActionBase<NavlineAction> {
 public:
  NavlineAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<NavlineAction>(name, config, deps),
        current_waypoint_index_(0),
        speed_(2.0),
        loops_(1),
        current_loop_(0),
        arrival_distance_(2.0) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Point>>("wayPoints", "航点列表"),
        BT::InputPort<double>("spd", 2.0, "飞行速度"),
        BT::InputPort<int>("loops", 1, "循环次数"),
        BT::InputPort<double>("arvDis", 2.0, "到点距离")
    };
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:

  std::vector<geometry_msgs::msg::Point> waypoints_;
  size_t current_waypoint_index_;
  double speed_;
  int loops_;
  int current_loop_;
  double arrival_distance_;

  bool loadParameters();

  bool sendNavigationCommand(const geometry_msgs::msg::Point &target);

  bool checkWaypointReached(const geometry_msgs::msg::Point &target);
};