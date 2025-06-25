#pragma once

#include <rclcpp/rclcpp.hpp>

#include "custom_msgs/srv/command_int.hpp"
#include "behavior_node/base_nodes.hpp"

class TraceAttackControl : public StatefulActionBase<TraceAttackControl> {
 public:
  TraceAttackControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase(name, config, deps) {}
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("frame", 1, "Frame to trace attack on"),
        BT::InputPort<int>("command", 0, "Command to execute on trace attack"),
        BT::InputPort<int>("current", 1, "Current value of trace attack")
    };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  rclcpp::Client<custom_msgs::srv::CommandInt>::SharedFuture future_;
};