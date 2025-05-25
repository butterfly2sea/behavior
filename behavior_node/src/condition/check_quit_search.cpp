#include "behavior_node/condition/check_quit_search.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include "behavior_node/common/data_manager.hpp"

BT::PortsList CheckQuitSearch::providedPorts() {
  return {};  // 不需要端口
}

BT::NodeStatus CheckQuitSearch::tick() {
  auto object_computation = DataManager::getInstance().getObjectComputation();
  if (uint8_t target = DataManager::getInstance().getTargetId(); target != 0) {
    for (auto obj : object_computation.objs) {
      if (obj.id == target) {
        return BT::NodeStatus::SUCCESS;  // 条件成功
      }
    }
  } else if (object_computation.objs.size() == 1) {
    DataManager::getInstance().setTargetId(object_computation.objs[0].id);
    return BT::NodeStatus::SUCCESS;  // 条件成功
  }
  return BT::NodeStatus::FAILURE;  // 条件失败
}