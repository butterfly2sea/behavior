#pragma once

#include <string>

#include <rclcpp/node.hpp>
#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/msg/point32.hpp>

#include "behavior_node/base_nodes.hpp"

// 设置目标点节点
class SetDestinationPoint : public SyncActionBase<SetDestinationPoint> {
 public:
  SetDestinationPoint(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : SyncActionBase<SetDestinationPoint>(name, config, deps) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};