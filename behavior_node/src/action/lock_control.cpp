#include "behavior_node/action/lock_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/base_enum.hpp"

#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"

BT::NodeStatus LockControl::onStart() {
  txtLog().info(THISMODULE "Lock control action started");
  start_time_ = std::chrono::steady_clock::now();
  int target_state{0};
  if (!getInput("state", target_state)) {
    txtLog().error(THISMODULE "Failed to get input state");
    return BT::NodeStatus::FAILURE;
  }
  if (ros()->isServiceReady(ros_interface::services::LOCK_UNLOCK)) {
    txtLog().info(THISMODULE "Service is ready");
    auto req = std::make_shared<custom_msgs::srv::CommandBool::Request>();
    req->value = (target_state == LockState::UNLOCK); // 解锁为true
    future_ = ros()->callService<custom_msgs::srv::CommandBool>(ros_interface::services::LOCK_UNLOCK, req);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus LockControl::onRunning() {
  txtLog().info(THISMODULE "Lock control action running");
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > timeout_) {
    txtLog().error(THISMODULE "Timeout waiting for service response");
    return BT::NodeStatus::FAILURE;
  }
  if (future_.valid() && future_.wait_for(std::chrono::milliseconds(5)) == std::future_status::ready) {
    auto response = future_.get();
    // 目前用response->success来判断是否成功，原版使用simpVehi的lock来判断是否成功
    if (response->success) {
      txtLog().info(THISMODULE "Lock command sent successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().error(THISMODULE "Lock command response failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void LockControl::onHalted() {
  txtLog().info(THISMODULE "Lock command halted");
}
