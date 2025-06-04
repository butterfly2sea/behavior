#pragma once

#include <string>

#include "behavior_node/base_nodes.hpp"

// 检查是否到达目标点节点
class CheckArriveDestination : public ConditionBase<CheckArriveDestination> {
 public:
  CheckArriveDestination(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : ConditionBase<CheckArriveDestination>(name, config, deps) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:
};
