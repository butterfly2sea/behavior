#include "behavior_node/action/flight_mode_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/ros_interface_definitions.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/mission_context.hpp"

BT::NodeStatus FlightModeControl::onStart() {
  txtLog().info(THISMODULE "Starting flight mode control");
  int target_mode = 0;
  if (!getInput("mode", target_mode)) {
    txtLog().error(THISMODULE "Failed to get input mode");
    return BT::NodeStatus::FAILURE;
  }
  // TODO:: 其实此处可以简化掉
  std::string altName("alt");
  if (!getInput<std::string>("altParam", altName)) {
    txtLog().error(THISMODULE "failed to get altParam name");
    return BT::NodeStatus::FAILURE;
  }
  // 从地面站的控制指令获取设置高度参数
  float takeoff_z = context()->getParameter(altName).get<float>();
  if (ros()->isServiceReady(ros_interface::services::SET_FLIGHT_MODE)) {
    txtLog().info(THISMODULE "Service is ready");
    auto req = std::make_shared<custom_msgs::srv::CommandLong::Request>();
    BT::Expected<float> param7 = getInput<float>("param7");
    if (param7) {// 此参数用于起飞模式时，起飞高度值
      req->param7 = param7.value();
    } else {
      req->param7 = takeoff_z;// 起飞高度由地面站设置，保存在此处
    }
    if (target_mode != int(FlightMode::Unknown)) {
      req->command = target_mode;
    }
    txtLog().info(THISMODULE "Sending command: %i param7: %f", req->command, req->param7);
    future_ = ros()->callService<custom_msgs::srv::CommandLong>(ros_interface::services::SET_FLIGHT_MODE, req);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FlightModeControl::onRunning() {
  txtLog().info(THISMODULE "Waiting for service response");
  if (!future_.valid()) {
    return BT::NodeStatus::FAILURE;
  }
  if (future_.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
    auto result = future_.get();
    if (result->success) {
      txtLog().info(THISMODULE "Mode change success");
      // 原来实现是读取simpleVehi中的flymd，此处直接读取CommandLong_Response_的success字段
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().error(THISMODULE "Mode change failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void FlightModeControl::onHalted() {
  txtLog().info(THISMODULE "Mode change halted");
}

