#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <custom_msgs/srv/command_bool.hpp>
#include "behavior_node/base_nodes.hpp"

// 锁定控制节点
class LockControl : public StatefulActionBase<LockControl> {
 public:
  LockControl(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : StatefulActionBase<LockControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("state", 1, "锁定状态 (0-锁定, 1-解锁)")
    };
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:
  rclcpp::Client<custom_msgs::srv::CommandBool>::SharedFuture future_;
  int target_state_{0};
  std::chrono::milliseconds timeout_{100000};
};

