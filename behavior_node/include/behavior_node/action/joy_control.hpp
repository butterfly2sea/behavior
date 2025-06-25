#pragma once

#include <mavros_msgs/msg/manual_control.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "behavior_node/base_nodes.hpp"

class JoyControl : public StatefulActionBase<JoyControl> {
 public:
  JoyControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase(name, config, deps) {}
  static BT::PortsList providedPorts() {
    return {};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  mavros_msgs::msg::ManualControl getManualControl(sensor_msgs::msg::Joy& joy_msg){
    mavros_msgs::msg::ManualControl manual_control;
    manual_control.x = std::clamp((joy_msg.buttons[1]-1500)*2,-1000,  1000);
    manual_control.y = std::clamp((joy_msg.buttons[0]-1500)*2,-1000,  1000);
    manual_control.z = std::clamp((joy_msg.buttons[2]-1500)*2,-1000,  1000);
    manual_control.r = std::clamp((joy_msg.buttons[3]-1500)*2,-1000,  1000);
    return manual_control;
  };

};