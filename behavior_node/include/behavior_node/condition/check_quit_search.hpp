#pragma once
#include <string>

#include <rclcpp/node.hpp>

#include <behaviortree_cpp/condition_node.h>
#include <custom_msgs/msg/object_computation.hpp>

// 检查是否退出搜索节点
class CheckQuitSearch : public BT::ConditionNode {
 public:
  CheckQuitSearch(const std::string &name, const BT::NodeConfig &config): BT::ConditionNode(name, config){};

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:
  std::string topic_name_{"inner/information/object_computation"};
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<custom_msgs::msg::ObjectComputation>> subscription_;
  std::shared_ptr<custom_msgs::msg::ObjectComputation> object_computation_{nullptr};
};
