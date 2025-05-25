#pragma once

#include <string>

#include <rclcpp/node.hpp>
#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/msg/point32.hpp>

// 设置目标点节点
class SetDestinationPoint : public BT::SyncActionNode {
 public:
  SetDestinationPoint(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};