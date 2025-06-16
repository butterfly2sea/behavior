#include "behavior_node/action/off_board_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"
#include "behavior_node/utils/utility.hpp"

BT::PortsList OffBoardControl::providedPorts() {
  return {
      BT::InputPort<float>("yaw"),
      BT::InputPort<float>("zoffset"),
      BT::InputPort<float>("fixed"),
  };
}

BT::NodeStatus OffBoardControl::tick() {
  txtLog().info(THISMODULE "OffBoardControl: ticking");
  auto vehicle_state = cache()->getVehicleState();
  custom_msgs::msg::OffboardCtrl ctrl;
  BT::Expected<float> yaw_input = getInput<float>("yaw");
  BT::Expected<float> zoffset = getInput<float>("zoffset");
  BT::Expected<float> fixed = getInput<float>("fixed");
  if (yaw_input) {
    ctrl.yaw = yaw_input.value();
  }
  ctrl.ordmask = OffBoardMask::LocCtrl + OffBoardMask::YawCtrl;//默认为位置+航向控制
  ctrl.x = vehicle_state->x / 1e3;//当前x
  ctrl.y = vehicle_state->y / 1e3;//当前y
  ctrl.z = vehicle_state->z / 1e3;//当前z
  if (zoffset) {
    ctrl.z += zoffset.value();
  } else if (fixed) {
    ctrl.z = fixed.value();
//  } else {
//    ctrl.z = FlightmodeCtrl::takeoffZ();
  }
  //如为垂起-固定翼则改变控制mask为行路点+空速
  if ((VehicleType::FixWing == vehicle_state->type) || (VehicleType::VtolFix == vehicle_state->type)) {
    ctrl.airspd = 0;//空速为0时飞控会使用最小空速
    ctrl.ordmask = OffBoardMask::LocCtrl + OffBoardMask::AirSpdCtrl;
  }
  utility::checkZValid(ctrl.z);//纠正起飞高度
  if (ros()->publish(ros_interface::topics::OFFBOARD_CONTROL, ctrl)) {
    txtLog().info(THISMODULE "OffBoardControl: published offboard control message:");
    return BT::NodeStatus::SUCCESS;
  } else {
    txtLog().error(THISMODULE "OffBoardControl: failed to publish offboard control message:");
    return BT::NodeStatus::FAILURE;
  }
}
