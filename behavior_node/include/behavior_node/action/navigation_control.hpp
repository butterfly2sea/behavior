#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <custom_msgs/srv/command_int.hpp>
#include "behavior_node/base_nodes.hpp"

// 导航控制节点
class NavigationControl : public StatefulActionBase<NavigationControl> {
 public:
  NavigationControl(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : StatefulActionBase<NavigationControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("frame", 1, "控制框架 (0-仅计算, 1-自主控制)"),
        BT::InputPort<int>("command", 0, "导航命令 (0-开始, 1-暂停, 2-恢复, 3-停止)")
    };
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:
  rclcpp::Client<custom_msgs::srv::CommandInt>::SharedFuture future_;
};