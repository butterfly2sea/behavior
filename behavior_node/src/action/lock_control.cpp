#include "behavior_node/action/lock_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/base_enum.hpp"

#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"

BT::PortsList LockControl::providedPorts() {
  return {
      BT::InputPort<int>("state", "锁定状态 (0-锁定, 1-解锁)"),
      BT::OutputPort<bool>("result", "控制结果")
  };
}

BT::NodeStatus LockControl::onStart() {
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
  if (future_.valid() && future_.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
    auto response = future_.get();
    // 目前用response->success来判断是否成功，原版使用simpVehi的lock来判断是否成功
    if (response->success) {
      txtLog().info(THISMODULE "Lock command sent successfully");
      setOutput("result", true);
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().error(THISMODULE "Lock command response failed");
      setOutput("result", false);
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void LockControl::onHalted() {
  txtLog().info(THISMODULE "Lock command halted");
  setOutput("result", false);
}
