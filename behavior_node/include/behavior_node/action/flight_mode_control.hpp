#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include <custom_msgs/srv/command_long.hpp>
#include "behavior_node/base_nodes.hpp"

// 飞行模式控制节点
class FlightModeControl : public StatefulActionBase<FlightModeControl> {
 public:
  FlightModeControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FlightModeControl>(name, config, deps) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:
  rclcpp::Client<custom_msgs::srv::CommandLong>::SharedFuture future_;
};