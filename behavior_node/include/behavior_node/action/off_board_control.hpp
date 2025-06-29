#pragma once

#include <string>

#include <rclcpp/node.hpp>
#include <behaviortree_cpp/action_node.h>

#include <custom_msgs/msg/offboard_ctrl.hpp>
#include "behavior_node/base_nodes.hpp"

// 外部控制节点
class OffBoardControl : public SyncActionBase<OffBoardControl> {
 public:
  OffBoardControl(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : SyncActionBase<OffBoardControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<custom_msgs::msg::OffboardCtrl>("ctrl", "控制指令"),
        BT::InputPort<float>("yaw", 0.0f, "偏航角(弧度)"),
        BT::InputPort<float>("zoffset", 0.0f, "Z轴偏移量(米)"),
        BT::InputPort<float>("fixed", 0.0f, "固定Z轴高度(米)")
    };
  }

  BT::NodeStatus tick() override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string topic_name_{"inner/control/offboard"};
  rclcpp::Publisher<custom_msgs::msg::OffboardCtrl>::SharedPtr pub_;

};
