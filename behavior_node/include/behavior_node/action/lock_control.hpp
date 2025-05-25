#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include <custom_msgs/srv/command_bool.hpp>

// 锁定控制节点
class LockControl : public BT::StatefulActionNode {
 public:
  LockControl(const std::string &name, const BT::NodeConfig &config,  std::shared_ptr<rclcpp::Node> node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  std::string service_name_{"inner/control/lock_unlock"};

  int target_state_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<custom_msgs::srv::CommandBool>::SharedPtr client_;
  rclcpp::Client<custom_msgs::srv::CommandBool>::SharedFuture future_;
};

