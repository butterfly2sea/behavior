#include "behavior_node/action/off_board_control.hpp"

OffBoardControl::OffBoardControl(const std::string &name,
                                 const BT::NodeConfig &config,
                                 const rclcpp::Node::SharedPtr &node)
    : BT::SyncActionNode(name, config),
      node_(node),
      pub_(node->create_publisher<custom_msgs::msg::OffboardCtrl>(topic_name_, 10)) {
}

BT::PortsList OffBoardControl::providedPorts() {
  return {
      BT::InputPort<custom_msgs::msg::OffboardCtrl>("ctrl"),
      BT::InputPort<float>("yaw")
  };
}

BT::NodeStatus OffBoardControl::tick() {
  BT::Expected<custom_msgs::msg::OffboardCtrl> ctrl_input = getInput<custom_msgs::msg::OffboardCtrl>("ctrl");
  BT::Expected<float> yaw_input = getInput<float>("yaw");
  if (!ctrl_input || !yaw_input) {
    return BT::NodeStatus::FAILURE;
  }
  custom_msgs::msg::OffboardCtrl ctrl = ctrl_input.value();
  ctrl.yaw = yaw_input.value();
  pub_->publish(ctrl);
  return BT::NodeStatus::SUCCESS;
}