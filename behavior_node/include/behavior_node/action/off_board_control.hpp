#pragma once

#include <string>

#include <rclcpp/node.hpp>
#include <behaviortree_cpp/action_node.h>

#include <custom_msgs/msg/offboard_ctrl.hpp>

// 外部控制节点
class OffBoardControl : public BT::SyncActionNode {
 public:
  OffBoardControl(const std::string &name, const BT::NodeConfig &config, const rclcpp::Node::SharedPtr& node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string topic_name_{"inner/control/offboard"};
  rclcpp::Publisher<custom_msgs::msg::OffboardCtrl>::SharedPtr pub_;

};
