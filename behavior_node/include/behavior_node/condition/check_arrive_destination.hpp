#pragma once

#include <string>
#include <custom_msgs/msg/offboard_ctrl.hpp>

#include "behavior_node/base_nodes.hpp"

// 检查是否到达目标点节点
class CheckArriveDestination : public ConditionBase<CheckArriveDestination> {
 public:
  CheckArriveDestination(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : ConditionBase<CheckArriveDestination>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<custom_msgs::msg::OffboardCtrl>("target", "目标位置"),
        BT::InputPort<float>("arvdis", 3.0f, "到达距离阈值(米)"),
        BT::InputPort<bool>("onlyz", false, "仅检查Z轴")
    };
  }

  BT::NodeStatus tick() override;

 private:
};
