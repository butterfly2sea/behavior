#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behavior_node/data/base_enum.hpp"

#include "behavior_node/base_nodes.hpp"

// 设置线路参数节点
class SetLineParameters : public SyncActionBase<SetLineParameters> {
 public:
  SetLineParameters(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : SyncActionBase<SetLineParameters>(name, config, deps) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:

};