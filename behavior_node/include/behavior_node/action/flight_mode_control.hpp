#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/srv/command_long.hpp>

#include "behavior_node/base_nodes.hpp"

// 飞行模式控制节点
class FlightModeControl : public StatefulActionBase<FlightModeControl> {
 public:
  FlightModeControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FlightModeControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("mode",
                           1,
                           "飞行模式 (1-手动, 2-定高, 3-定点, 4-起飞, 5-降落, 6-等待, 7-任务, 8-返航, 9-外部, 10-增稳)"),
        BT::InputPort<float>("param7", 0.0f, "起飞高度(米，负值表示相对地面高度)"),
        BT::InputPort<std::string>("altParam", "alt", "高度参数名称")
    };
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:
  rclcpp::Client<custom_msgs::srv::CommandLong>::SharedFuture future_;
};