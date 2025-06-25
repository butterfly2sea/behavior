#include "behavior_node/action/joy_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"

BT::NodeStatus JoyControl::onStart() {
  txtLog().info(THISMODULE "JoyControl action started");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus JoyControl::onRunning() {
  if (!cache()->isJoyControlValid(2)) {
    return BT::NodeStatus::FAILURE;
  }
  auto joy_control = cache()->getJoyControl();
  ros()->publish(ros_interface::topics::MANUAL_CONTROL, getManualControl(*joy_control));
  return BT::NodeStatus::RUNNING;
}

void JoyControl::onHalted() {
  txtLog().info(THISMODULE "JoyControl action halted");
}