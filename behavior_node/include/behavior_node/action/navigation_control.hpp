#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <custom_msgs/srv/command_int.hpp>
#include "behavior_node/base_nodes.hpp"

// 导航控制节点
class NavigationControl : public StatefulActionBase<NavigationControl> {
 public:
  NavigationControl(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : StatefulActionBase<NavigationControl>(name, config, deps) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:
  rclcpp::Client<custom_msgs::srv::CommandInt>::SharedFuture future_;
};