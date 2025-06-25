#include "behavior_node/action/trace_attack_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"

BT::NodeStatus TraceAttackControl::onStart() {
  txtLog().info(THISMODULE "TraceAttackControl action started");
  start_time_ = std::chrono::steady_clock::now();
  if (ros()->isServiceReady(ros_interface::services::LOCK_UNLOCK)) {
    txtLog().info(THISMODULE "Service is ready");
    auto request = std::make_shared<custom_msgs::srv::CommandInt::Request>();

    BT::Expected<uint8_t> frame = getInput<int>("frame");
    BT::Expected<uint16_t> command = getInput<int>("command");
    BT::Expected<uint16_t> current = getInput<int>("current");
    //优先从参数获取frame值，否则为1（节点自己进行offboard控制）
    if (frame) {
      request->frame = frame.value();
    } else
      request->frame = 1;
    //优先从参数获取command值，否则为0（开始执行跟踪打击）
    if (command) {
      request->command = command.value();
    } else
      request->command = 0;
    //优先从参数获取current值，否则地面站指定
    if (current) {
      request->current = current.value();
    } else
      request->current = context()->getTraceAttackType();
    future_ = ros()->callService<custom_msgs::srv::CommandInt>(ros_interface::services::GUIDANCE_SWITCH, request);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TraceAttackControl::onRunning() {
  txtLog().info(THISMODULE "TraceAttackControl action running");
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > timeout_) {
    txtLog().error(THISMODULE "Timeout waiting for service response");
    return BT::NodeStatus::FAILURE;
  }
  if (future_.valid() && future_.wait_for(std::chrono::milliseconds(5)) == std::future_status::ready) {
    auto response = future_.get();
    // 目前用response->success来判断是否成功
    if (response->success) {
      txtLog().info(THISMODULE "trace attack command sent successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().error(THISMODULE "trace attack command response failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void TraceAttackControl::onHalted() {
  txtLog().info(THISMODULE "TraceAttackControl action halted");
}