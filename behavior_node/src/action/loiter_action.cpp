#include "behavior_node/action/loiter_action.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"

BT::NodeStatus LoiterAction::onStart() {
// 获取悬停时间
  auto duration_opt = getInput<int>("duration");
  if (duration_opt) {
    duration_seconds_ = duration_opt.value();
  } else if (context()->hasParameter("duration")) {
    duration_seconds_ = std::stoi(context()->getParameter("duration").get<std::string>());
  }

  txtLog().info(THISMODULE "Starting loiter for %d seconds", duration_seconds_);

  if (!sendLoiterCommand()) {
    return BT::NodeStatus::FAILURE;
  }

  hover_start_ = std::chrono::steady_clock::now();
  start_time_ = hover_start_;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LoiterAction::onRunning() {
  auto elapsed = std::chrono::steady_clock::now() - hover_start_;
  auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

  if (elapsed_seconds >= duration_seconds_) {
    txtLog().info(THISMODULE "Loiter duration completed");
    return BT::NodeStatus::SUCCESS;
  }

// 继续发送悬停指令
  sendLoiterCommand();
  return BT::NodeStatus::RUNNING;
}

void LoiterAction::onHalted() {
  txtLog().info(THISMODULE "Loiter action halted");
}

bool LoiterAction::sendLoiterCommand() {
  try {
    // 获取当前位置
    auto vehicle_state = cache()->getVehicleState();
    if (!vehicle_state) return false;

    // 发送保持当前位置的offboard指令
    custom_msgs::msg::OffboardCtrl ctrl_msg;
    ctrl_msg.x = vehicle_state->x;
    ctrl_msg.y = vehicle_state->y;
    ctrl_msg.z = vehicle_state->z;
    ctrl_msg.yaw = 0.0;

    ros()->publishOffboardControl(ctrl_msg);
    return true;
  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Failed to send loiter command: %s", e.what());
    return false;
  }
}
