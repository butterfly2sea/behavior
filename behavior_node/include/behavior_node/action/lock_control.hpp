#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <custom_msgs/srv/command_bool.hpp>
#include "base_nodes.hpp"

// 锁定控制节点
class LockControl : public StatefulActionBase<LockControl> {
 public:
  LockControl(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : StatefulActionBase<LockControl>(name, config, deps) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  rclcpp::Client<custom_msgs::srv::CommandBool>::SharedFuture future_;
};

