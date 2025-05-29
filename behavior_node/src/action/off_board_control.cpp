#include "behavior_node/action/off_board_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"

BT::PortsList OffBoardControl::providedPorts() {
  return {
      BT::InputPort<custom_msgs::msg::OffboardCtrl>("ctrl"),
      BT::InputPort<float>("yaw")
  };
}

BT::NodeStatus OffBoardControl::tick() {
  BT::Expected<custom_msgs::msg::OffboardCtrl> ctrl_input = getInput<custom_msgs::msg::OffboardCtrl>("ctrl");
  BT::Expected<float> yaw_input = getInput<float>("yaw");
  if (!ctrl_input || !yaw_input) {
    txtLog().error(THISMODULE "OffBoardControl: missing input with port name: %s  %s", ctrl_input ? "" : "ctrl",
                   yaw_input ? "" : "yaw");
    return BT::NodeStatus::FAILURE;
  }
  custom_msgs::msg::OffboardCtrl ctrl = ctrl_input.value();
  ctrl.yaw = yaw_input.value();
  if (ros()->publish(ros_interface::topics::OFFBOARD_CONTROL, ctrl)) {
    txtLog().info(THISMODULE "OffBoardControl: published offboard control message:");
    return BT::NodeStatus::SUCCESS;
  } else {
    txtLog().error(THISMODULE "OffBoardControl: failed to publish offboard control message:");
    return BT::NodeStatus::FAILURE;
  }
}
