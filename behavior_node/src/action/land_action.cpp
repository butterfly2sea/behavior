#include "behavior_node/action/land_action.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"

BT::NodeStatus LandAction::onStart() {
  txtLog().info(THISMODULE "Starting landing sequence");

  if (!sendLandCommand()) {
    return BT::NodeStatus::FAILURE;
  }

  land_initiated_ = true;
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LandAction::onRunning() {
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > timeout_) {
    txtLog().error(THISMODULE "Landing timeout after %ld ms", timeout_.count());
    return BT::NodeStatus::FAILURE;
  }

// 检查是否已着陆
  if (checkLandingComplete()) {
    txtLog().info(THISMODULE "Landing completed successfully");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void LandAction::onHalted() {
  land_initiated_ = false;
  txtLog().info(THISMODULE "Land action halted");
}

bool LandAction::sendLandCommand() {
  try {
    return ros()->callLandService();
  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Failed to send land command: %s", e.what());
    return false;
  }
}

bool LandAction::checkLandingComplete() {
  auto vehicle_state = cache()->getVehicleState();
  if (!vehicle_state) return false;

  // 检查是否在地面附近且锁定
  double altitude = -vehicle_state->z / 1000.0;
  bool on_ground = altitude < 1.0; // 1米以下认为在地面
  bool locked = vehicle_state->lock == 0; // 0表示锁定

  return on_ground && locked;
}
