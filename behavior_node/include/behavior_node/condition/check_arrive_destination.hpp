#pragma once

#include <string>

#include <behaviortree_cpp/condition_node.h>


// 检查是否到达目标点节点
class CheckArriveDestination : public BT::ConditionNode {
 public:
  CheckArriveDestination(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:
};
