#include "behavior_node/condition/check_arrive_destination.hpp"

#include <log/Logger.hpp>

#include "behavior_node/utils/utility.hpp"
#include "behavior_node/data/data_cache.hpp"

BT::PortsList CheckArriveDestination::providedPorts() {
  return {
      BT::InputPort<custom_msgs::msg::OffboardCtrl>("target", "目标位置"),
      BT::InputPort<float>("arvdis", 0.2, "到达距离阈值"),
      BT::InputPort<bool>("onlyz", false, "仅检查Z轴")
  };
}

BT::NodeStatus CheckArriveDestination::tick() {
  // 获取目标位置
  custom_msgs::msg::OffboardCtrl target;
  float arrive_distance = 1.0;
  bool only_check_z = false;
  getInput("target", target);
  getInput("arvdis", arrive_distance);
  getInput("onlyz", only_check_z);

  if (!cache()->isVehicleStateValid()) return BT::NodeStatus::FAILURE;

  auto current = cache()->getVehicleState();
  if (only_check_z) {
    if (utility::getDisFrmLoc(current->x, current->y, target.x, target.y) < arrive_distance) {
      txtLog().info(THISMODULE "arrive destination");
      return BT::NodeStatus::SUCCESS;
    }
    txtLog().info(THISMODULE "not arrive destination yet");
    return BT::NodeStatus::FAILURE;
  } else {
    if (utility::getDisFrmLoc(current->x, current->y, current->z, target.x, target.y, target.z) < arrive_distance) {
      txtLog().info(THISMODULE "arrive destination");
      return BT::NodeStatus::SUCCESS;
    }
    txtLog().info(THISMODULE "not arrive destination yet");
    return BT::NodeStatus::FAILURE;
  }

}
