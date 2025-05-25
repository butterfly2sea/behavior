#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include <custom_msgs/srv/command_long.hpp>

// 飞行模式控制节点
class FlightModeControl : public BT::StatefulActionNode {
 public:
  FlightModeControl(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  std::string service_name_{"inner/control/set_flymode"};
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Client<custom_msgs::srv::CommandLong>> client_;
  rclcpp::Client<custom_msgs::srv::CommandLong>::SharedFuture future_;
};