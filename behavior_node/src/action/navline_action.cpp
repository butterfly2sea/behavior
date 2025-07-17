#include "behavior_node/action/navline_action.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"

BT::NodeStatus NavlineAction::onStart() {
// 从参数获取航点列表和飞行参数
  if (!loadParameters()) {
    return BT::NodeStatus::FAILURE;
  }

  txtLog().info(THISMODULE
                "Starting navline with %zu waypoints, %d loops, speed %.1f m/s",
                waypoints_.size(),
                loops_,
                speed_);

  current_waypoint_index_ = 0;
  current_loop_ = 0;
  start_time_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavlineAction::onRunning() {
  if (current_loop_ >= loops_) {
    txtLog().info(THISMODULE "All navline loops completed");
    return BT::NodeStatus::SUCCESS;
  }

  if (current_waypoint_index_ >= waypoints_.size()) {
// 完成一圈，开始下一圈
    current_loop_++;
    current_waypoint_index_ = 0;
    txtLog().info(THISMODULE "Completed loop %d/%d", current_loop_, loops_);

    if (current_loop_ >= loops_) {
      return BT::NodeStatus::SUCCESS;
    }
  }

// 飞向当前航点
  const auto &target_waypoint = waypoints_[current_waypoint_index_];

// 发送导航指令
  if (!sendNavigationCommand(target_waypoint)) {
    return BT::NodeStatus::FAILURE;
  }

// 检查是否到达当前航点
  if (checkWaypointReached(target_waypoint)) {
    txtLog().info(THISMODULE "Reached waypoint %zu", current_waypoint_index_);
    current_waypoint_index_++;
  }

  return BT::NodeStatus::RUNNING;
}

void NavlineAction::onHalted() {
  txtLog().info(THISMODULE "Navline action halted");
}

bool NavlineAction::loadParameters() {
  // 从任务上下文加载参数
  if (!context()->hasParameter("wayPoints") || !context()->hasParameter("spd") || !context()->hasParameter("loops")
      || !context()->hasParameter("arvDis")) {
    txtLog().error(THISMODULE "Failed to load navline parameters");
    return false;
  }
  auto waypoints_param = context()->getParameter("wayPoints");
  auto speed_param = context()->getParameter("spd");
  auto loops_param = context()->getParameter("loops");
  auto arvdis_param = context()->getParameter("arvDis");


  // 解析航点参数 (这里需要根据实际JSON格式解析)
  if (waypoints_param.is_array()) {
    for (const auto &wp : waypoints_param) {
      geometry_msgs::msg::Point point;
      point.x = wp["x_lat"].get<double>();
      point.y = wp["y_lon"].get<double>();
      point.z = wp["z_alt"].get<double>();
      waypoints_.push_back(point);
    }
  }
  speed_ = std::stod(speed_param.get<std::string>());

  loops_ = std::stoi(loops_param.get<std::string>());

  arrival_distance_ = std::stod(arvdis_param.get<std::string>());

  return !waypoints_.empty();
}

bool NavlineAction::sendNavigationCommand(const geometry_msgs::msg::Point &target) {
  try {
    custom_msgs::msg::OffboardCtrl ctrl_msg;
    ctrl_msg.x = static_cast<int32_t>(target.x * 1000000); // 转换为微度
    ctrl_msg.y = static_cast<int32_t>(target.y * 1000000);
    ctrl_msg.z = static_cast<int32_t>(-target.z * 1000); // 转换为毫米，注意Z轴方向
    ctrl_msg.yaw = 0.0;

    ros()->publishOffboardControl(ctrl_msg);
    return true;
  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Failed to send navigation command: %s", e.what());
    return false;
  }
}

bool NavlineAction::checkWaypointReached(const geometry_msgs::msg::Point &target) {
  auto vehicle_state = cache()->getVehicleState();
  if (!vehicle_state) return false;

  // 计算与目标点的距离
  double dx = (vehicle_state->x / 1000000.0) - target.x; // 转换为度
  double dy = (vehicle_state->y / 1000000.0) - target.y;
  double dz = (-vehicle_state->z / 1000.0) - target.z; // 转换为米

  double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  return distance < (arrival_distance_ / 111000.0); // 粗略转换为度
}
