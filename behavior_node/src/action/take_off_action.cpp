#include "behavior_node/action/take_off_action.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"

BT::NodeStatus TakeOffAction::onStart() {
  if (context()->hasParameter("alt")) {
    target_altitude_ = std::stod(context()->getParameter("alt").get<std::string>());
  } else {
    txtLog().error(THISMODULE "Takeoff action failed: no altitude provided");
    return BT::NodeStatus::FAILURE;
  }
  txtLog().info(THISMODULE "Starting takeoff to altitude: %.1f meters", target_altitude_);

  // 发送起飞指令
  if (!sendTakeoffCommand()) {
    return BT::NodeStatus::FAILURE;
  }

  takeoff_initiated_ = true;
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TakeOffAction::onRunning() {
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > timeout_) {
    txtLog().error(THISMODULE "Takeoff timeout after %ld ms", timeout_.count());
    return BT::NodeStatus::FAILURE;
  }

  // 检查是否到达目标高度
  if (
      checkAltitudeReached()) {
    txtLog().info(THISMODULE "Takeoff completed successfully");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void TakeOffAction::onHalted() {
  takeoff_initiated_ = false;
  txtLog().info(THISMODULE "Takeoff action halted");
}

bool TakeOffAction::sendTakeoffCommand() {
  try {
    // 通过服务调用发送起飞指令
    return ros()->callTakeoffService(target_altitude_);
  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Failed to send takeoff command: %s", e.what());
    return false;
  }
}

bool TakeOffAction::checkAltitudeReached() {
  auto vehicle_state = cache()->getVehicleState();
  if (!vehicle_state) return false;

  double current_alt = -vehicle_state->z / 1000.0; // 转换为米
  double tolerance = 2.0; // 2米容差

  return std::abs(current_alt - target_altitude_) < tolerance;
}