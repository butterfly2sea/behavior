#include "behavior_node/condition/check_quit_search.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"

BT::PortsList CheckQuitSearch::providedPorts() {
  return {};  // 不需要端口
}

BT::NodeStatus CheckQuitSearch::tick() {
  txtLog().info(THISMODULE "check quit search tick");
  auto object_computation = cache()->getDetectedObjects();
  if (uint8_t target = context()->getTargetId(); target != 0) {
    for (auto obj : object_computation->objs) {
      if (obj.id == target) {
        return BT::NodeStatus::SUCCESS;  // 条件成功
      }
    }
  } else if (object_computation->objs.size() == 1) {
    context()->setTargetId(object_computation->objs[0].id);
    return BT::NodeStatus::SUCCESS;  // 条件成功
  }
  return BT::NodeStatus::FAILURE;  // 条件失败
}