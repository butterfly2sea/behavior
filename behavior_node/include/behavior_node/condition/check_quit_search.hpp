#pragma once
#include <string>

#include <rclcpp/node.hpp>

#include <behaviortree_cpp/condition_node.h>
#include <custom_msgs/msg/object_computation.hpp>
#include "base_nodes.hpp"

// 检查是否退出搜索节点
class CheckQuitSearch : public ConditionBase<CheckQuitSearch> {
 public:
  CheckQuitSearch(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : ConditionBase<CheckQuitSearch>(name, config, deps) {};

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:
  std::string topic_name_{"inner/information/object_computation"};
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Subscription<custom_msgs::msg::ObjectComputation>> subscription_;
  std::shared_ptr<custom_msgs::msg::ObjectComputation> object_computation_{nullptr};
};
