#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include <std_msgs/msg/string.hpp>
#include "behavior_node/common/data_manager.hpp"

class BehaviorNode : public rclcpp::Node {
 public:
  explicit BehaviorNode(const std::string &node_name, std::string xml_file_dir);
  ~BehaviorNode() override;

 private:

  // 参数
  std::string config_dir_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  DataManager &data_manager_{DataManager::getInstance()};
};