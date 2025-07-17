#include "behavior_node/action/rtl_direct_action.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"

BT::NodeStatus RtlDirectAction::onStart() {
  txtLog().info(THISMODULE "Starting RTL (Return to Launch)");

  if (!sendRtlCommand()) {
    return BT::NodeStatus::FAILURE;
  }

  rtl_initiated_ = true;
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RtlDirectAction::onRunning() {
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::minutes(5)) { // 5分钟超时
    txtLog().error(THISMODULE "RTL timeout after 5 minutes");
    return BT::NodeStatus::FAILURE;
  }

// 检查是否已返回起飞点
  if (checkRtlComplete()) {
    txtLog().info(THISMODULE "RTL completed successfully");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void RtlDirectAction::onHalted() {
  rtl_initiated_ = false;
  txtLog().info(THISMODULE "RTL action halted");
}

bool RtlDirectAction::sendRtlCommand() {
  try {
    return ros()->callRtlService();
  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Failed to send RTL command: %s", e.what());
    return false;
  }
}

bool RtlDirectAction::checkRtlComplete() {
  auto vehicle_state = cache()->getVehicleState();
  if (!vehicle_state) return false;

  // 获取起飞点位置
  auto home_position = context()->getHomePoint();

  // 检查是否接近起飞点
  double dx = (vehicle_state->x - home_position.x) / 1000.0;
  double dy = (vehicle_state->y - home_position.y) / 1000.0;
  double distance = std::sqrt(dx * dx + dy * dy);

  return distance < 5.0; // 5米容差
}
