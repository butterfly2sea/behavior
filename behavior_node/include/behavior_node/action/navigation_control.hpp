#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include <custom_msgs/srv/command_int.hpp>

// 导航控制节点
class NavigationControl : public BT::StatefulActionNode {
 public:
  NavigationControl(const std::string &name,
                    const BT::NodeConfig &config,
                    std::shared_ptr<rclcpp::Node> node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:

  std::string service_name_{"inner/control/form_switch"};
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<custom_msgs::srv::CommandInt>::SharedPtr client_;
  rclcpp::Client<custom_msgs::srv::CommandInt>::SharedFuture future_;
};