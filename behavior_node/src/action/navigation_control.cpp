#include "behavior_node/action/navigation_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"

BT::PortsList NavigationControl::providedPorts() {
  return {
      BT::InputPort<int>("frame", 0, "1：节点自己进行offboard控制，0：节点只进行计算不进行实际控制"),
      BT::InputPort<int>("command", "导航命令 (0-开始, 1-暂停, 2-恢复, 3-停止)")
  };
}

BT::NodeStatus NavigationControl::onStart() {
  txtLog().info(THISMODULE "NavigationControl: Starting navigation control");
  uint8_t frame = 1; // 节点自己进行offboard控制
  uint8_t command = 0; // 默认使用开始编队飞行
  if (!getInput("frame", frame)) frame = 1; // 没有参数则使用1（节点自己进行offboard控制）
  if (!getInput("command", command)) command = 0; // 没有参数则使用0（开始编队飞行）
  auto req = std::make_shared<custom_msgs::srv::CommandInt::Request>();
  req->frame = frame;
  req->command = command;
  if (ros()->isServiceReady(ros_interface::services::FORMATION_SWITCH)) {
    txtLog().info(THISMODULE "NavigationControl: Navigation service is ready");
    future_ = ros()->callService<custom_msgs::srv::CommandInt>(ros_interface::services::FORMATION_SWITCH, req);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigationControl::onRunning() {
  txtLog().info(THISMODULE "NavigationControl: Navigation control running");
  if (future_.valid() && future_.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
    auto response = future_.get();
    if (response->success) {
      txtLog().info(THISMODULE "Navigation command sent successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().error(THISMODULE "Navigation command failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void NavigationControl::onHalted() {
  txtLog().info(THISMODULE "Navigation command halted");
}
