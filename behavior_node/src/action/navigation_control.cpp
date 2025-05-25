#include "behavior_node/action/navigation_control.hpp"

#include <log/Logger.hpp>

NavigationControl::NavigationControl(const std::string &name,
                                     const BT::NodeConfig &config,
                                     std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      client_(node->create_client<custom_msgs::srv::CommandInt>(service_name_)) {
}

BT::PortsList NavigationControl::providedPorts() {
  return {
      BT::InputPort<int>("frame", 0, "坐标系 (0-LOCAL_NED, 1-BODY_NED, 2-LOCAL_ENU, 3-GLOBAL)"),
      BT::InputPort<int>("command", "导航命令 (0-开始, 1-暂停, 2-恢复, 3-停止)")
  };
}

BT::NodeStatus NavigationControl::onStart() {
  uint8_t frame = 1; // 默认使用BODY_NED坐标系
  uint8_t command = 0; // 默认使用开始编队飞行
  if (!getInput("frame", frame)) frame = 1; // 没有参数则使用1（节点自己进行offboard控制）
  if (!getInput("command", command)) command = 0; // 没有参数则使用0（开始编队飞行）
  auto req = std::make_shared<custom_msgs::srv::CommandInt::Request>();
  req->frame = frame;
  req->command = command;
  if (client_->wait_for_service(std::chrono::milliseconds(50)) && client_->service_is_ready()) {
    txtLog().info(THISMODULE "NavigationControl: Navigation service is ready");
    future_ = client_->async_send_request(req);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigationControl::onRunning() {
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
