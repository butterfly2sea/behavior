#pragma once
#include <string>

#include <rclcpp/node.hpp>

#include <behaviortree_cpp/condition_node.h>
#include <custom_msgs/msg/object_computation.hpp>
#include "behavior_node/base_nodes.hpp"

// 检查是否退出搜索节点
class CheckQuitLineLoop : public ConditionBase<CheckQuitLineLoop> {
 public:
  CheckQuitLineLoop(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : ConditionBase<CheckQuitLineLoop>(name, config, deps) {};

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override;
};
