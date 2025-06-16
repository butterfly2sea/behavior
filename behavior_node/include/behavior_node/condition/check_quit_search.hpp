#pragma once
#include <string>

#include <rclcpp/node.hpp>

#include <behaviortree_cpp/condition_node.h>
#include <custom_msgs/msg/object_computation.hpp>
#include "behavior_node/base_nodes.hpp"

// 检查是否退出搜索节点
class CheckQuitSearch : public ConditionBase<CheckQuitSearch> {
 public:
  CheckQuitSearch(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : ConditionBase<CheckQuitSearch>(name, config, deps) {};

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override;
};
