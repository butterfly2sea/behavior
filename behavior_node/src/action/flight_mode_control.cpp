#include "behavior_node/action/flight_mode_control.hpp"

#include <log/Logger.hpp>

#include "behavior_node/common/data_manager.hpp"

FlightModeControl::FlightModeControl(const std::string &name,
                                     const BT::NodeConfig &config,
                                     std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      client_(node->create_client<custom_msgs::srv::CommandLong>(service_name_)) {
}

BT::PortsList FlightModeControl::providedPorts() {
  return {
      BT::InputPort<int>("mode", "飞行模式，取值范围[1,8]，分别对应FlightMode枚举值"),
      BT::InputPort<float>("param7", "起飞高度"),
      BT::InputPort<std::string>("altParam", "高度参数名称")
  };
}

BT::NodeStatus FlightModeControl::onStart() {
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
  float takeoff_z = DataManager::getInstance().getJsonParams(altName).get<float>();
  if (client_->wait_for_service(std::chrono::milliseconds(50)) && client_->service_is_ready()) {
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
    future_ = client_->async_send_request(req);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FlightModeControl::onRunning() {
  if (future_.valid() && future_.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
    auto result = future_.get();
    if (result->success) {
      txtLog().info(THISMODULE "Mode change success");
      // 原来实现是读取simpVehi中的flymd，此处直接读取CommandLong_Response_的success字段
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().error(THISMODULE "Mode change failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void FlightModeControl::onHalted() {
  RCLCPP_INFO(rclcpp::get_logger("FlightModeControl"), "Mode change halted");
}
