#include "behavior_node/actions/navigation_actions.hpp"

#include <cmath>

// ============= SetLineParameters 实现 =============

BT::NodeStatus SetLineParameters::execute() {
  int type = 127; // 默认设置所有参数
  float anti_dis = 8.0f;

  getInputValue<int>("type").assign_to(type);
  getInputValue<float>("antiDis").assign_to(anti_dis);

  logInfo("Setting line parameters with type mask: %d", type);

  try {
    // 发布防撞距离
    if (type & 1) {
      publishAntiCollisionDistance(anti_dis);
    }

    // 设置载具类型
    if (type & 2) {
      std::string vehi_param;
      if (getInputValue<std::string>("vehiTypParam").assign_to(vehi_param)) {
        auto vehi_type_opt = context()->getParameter(vehi_param);
        if (vehi_type_opt.has_value()) {
          VehicleType vtype = static_cast<VehicleType>(vehi_type_opt->get<int>());
          publishVehicleType(vtype);
        }
      }
    }

    // 设置速度
    if (type & 4) {
      std::string spd_param;
      if (getInputValue<std::string>("spdParam").assign_to(spd_param)) {
        auto speed_opt = context()->getParameter(spd_param);
        if (speed_opt.has_value()) {
          publishSpeed(speed_opt->get<float>());
        }
      }
    }

    // 设置到点距离
    if (type & 8) {
      std::string dis_param;
      if (getInputValue<std::string>("disParam").assign_to(dis_param)) {
        auto distance_opt = context()->getParameter(dis_param);
        if (distance_opt.has_value()) {
          publishArrivalDistance(distance_opt->get<float>());
        }
      }
    }

    // 设置点类型
    if (type & 16) {
      std::string pt_param;
      if (getInputValue<std::string>("ptTypParam").assign_to(pt_param)) {
        auto pt_type_opt = context()->getParameter(pt_param);
        if (pt_type_opt.has_value()) {
          PointType ptype = static_cast<PointType>(pt_type_opt->get<int>());
          publishPointTag(ptype);
        }
      }
    }

    // 设置航点
    if (type & 32) {
      std::string pts_param;
      if (getInputValue<std::string>("ptsParam").assign_to(pts_param)) {
        auto waypoints_opt = context()->getParameter(pts_param);
        if (waypoints_opt.has_value()) {
          // 从JSON解析航点
          geometry_msgs::msg::Polygon waypoints;
          // 这里需要JSON到Polygon的转换逻辑
          publishWaypoints(waypoints);
        }
      }
    }

    // 设置循环次数
    if (type & 64) {
      std::string loops_param;
      if (getInputValue<std::string>("lpsParam").assign_to(loops_param)) {
        auto loops_opt = context()->getParameter(loops_param);
        if (loops_opt.has_value()) {
          publishLoops(loops_opt->get<uint32_t>());
        }
      }
    }

    logInfo("Line parameters set successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    logError("Failed to set line parameters: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void SetLineParameters::publishVehicleType(VehicleType type) {
  ros()->publishVehicleType(static_cast<uint8_t>(type));
}

void SetLineParameters::publishSpeed(float speed) {
  ros()->publishGroupSpeed(speed);
}

void SetLineParameters::publishArrivalDistance(float distance) {
  context()->setArrivalDistance(distance);
}

void SetLineParameters::publishAntiCollisionDistance(float distance) {
  ros()->publishAntiCollisionDistance(distance);
  context()->setAntiCollisionDistance(distance);
}

void SetLineParameters::publishPointTag(PointType tag) {
  ros()->publishPointTag(static_cast<uint8_t>(tag));
}

void SetLineParameters::publishWaypoints(const geometry_msgs::msg::Polygon& waypoints) {
  ros()->publishNavline(waypoints);
  context()->setWaypoints(waypoints);
}

void SetLineParameters::publishLoops(uint32_t loops) {
  ros()->publishGroupLoops(loops);
  context()->setLoopCount(loops);
}

// ============= SetFormationOffset 实现 =============

BT::NodeStatus SetFormationOffset::execute() {
  std::string offset_param;
  if (!getInputValue<std::string>("offsetParam").assign_to(offset_param)) {
    logError("Failed to get offsetParam");
    return BT::NodeStatus::FAILURE;
  }

  auto offset_opt = context()->getParameter(offset_param);
  if (!offset_opt.has_value()) {
    logError("Formation offset parameter not found: %s", offset_param.c_str());
    return BT::NodeStatus::FAILURE;
  }

  try {
    // 从JSON转换为Polygon
    geometry_msgs::msg::Polygon offsets;
    // 这里需要JSON到Polygon的转换逻辑

    ros()->publishGroupOffset(offsets);
    context()->setFormationOffsets(offsets);

    logInfo("Formation offset set successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    logError("Failed to set formation offset: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ============= NavlineAction 实现 =============

BT::NodeStatus NavlineAction::onActionStart() {
  if (!getInputValue<geometry_msgs::msg::Polygon>("wayPoints").assign_to(waypoints_)) {
    logError("Failed to get waypoints parameter");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInputValue<float>("spd").assign_to(speed_)) {
    speed_ = 5.0f;
  }

  if (!getInputValue<uint32_t>("loops").assign_to(total_loops_)) {
    total_loops_ = 1;
  }

  if (!getInputValue<float>("arvDis").assign_to(arrival_distance_)) {
    arrival_distance_ = 5.0f;
  }

  if (waypoints_.points.empty()) {
    logError("No waypoints provided");
    return BT::NodeStatus::FAILURE;
  }

  current_loop_ = 0;
  current_waypoint_index_ = 0;
  waypoint_start_time_ = std::chrono::steady_clock::now();

  logInfo("Starting navline mission with %zu waypoints, %u loops, speed %.1f m/s",
          waypoints_.points.size(), total_loops_, speed_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavlineAction::onActionRunning() {
  if (current_loop_ >= total_loops_) {
    logInfo("Navline mission completed");
    return BT::NodeStatus::SUCCESS;
  }

  auto current_waypoint = getCurrentWaypoint();

  // 检查是否到达当前航点
  if (isAtWaypoint(current_waypoint)) {
    logInfo("Reached waypoint %zu of loop %u", current_waypoint_index_, current_loop_);

    if (hasNextWaypoint()) {
      moveToNextWaypoint();
    } else {
      // 完成当前循环
      current_loop_++;
      current_waypoint_index_ = 0;
      logInfo("Completed loop %u/%u", current_loop_, total_loops_);
    }

    waypoint_start_time_ = std::chrono::steady_clock::now();
  }

  // 发送控制指令
  sendOffboardControl(getCurrentWaypoint());

  // 检查超时
  if (isTimeout()) {
    logError("Navline mission timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void NavlineAction::onActionHalted() {
  logInfo("Navline mission halted");
}

bool NavlineAction::isAtWaypoint(const geometry_msgs::msg::Point32& waypoint) {
  auto current_pos = cache()->getCurrentPosition();

  double distance = std::sqrt(
      std::pow(current_pos.x - waypoint.x, 2) +
          std::pow(current_pos.y - waypoint.y, 2) +
          std::pow(current_pos.z - waypoint.z, 2));

  return distance < arrival_distance_;
}

geometry_msgs::msg::Point32 NavlineAction::getCurrentWaypoint() {
  if (current_waypoint_index_ < waypoints_.points.size()) {
    return waypoints_.points[current_waypoint_index_];
  }
  return geometry_msgs::msg::Point32();
}

bool NavlineAction::hasNextWaypoint() {
  return current_waypoint_index_ + 1 < waypoints_.points.size();
}

void NavlineAction::moveToNextWaypoint() {
  current_waypoint_index_++;
}

void NavlineAction::sendOffboardControl(const geometry_msgs::msg::Point32& target) {
  custom_msgs::msg::OffboardCtrl msg;
  msg.frame = 1; // LOCAL_NED
  msg.type_mask = 0;
  msg.x = target.x;
  msg.y = target.y;
  msg.z = target.z;
  msg.vx = speed_;
  msg.yaw = 0.0f;

  ros()->publishOffboardControl(msg);
}

// ============= GoToDestination 实现 =============

BT::NodeStatus GoToDestination::onActionStart() {
  if (!getInputValue<geometry_msgs::msg::Point>("target").assign_to(target_point_)) {
    logError("Failed to get target parameter");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInputValue<float>("speed").assign_to(speed_)) {
    speed_ = 5.0f;
  }

  if (!getInputValue<float>("arrival_distance").assign_to(arrival_distance_)) {
    arrival_distance_ = 3.0f;
  }

  float yaw_input = -999.0f;
  getInputValue<float>("yaw").assign_to(yaw_input);
  use_yaw_ = (yaw_input != -999.0f);
  target_yaw_ = yaw_input;

  logInfo("Going to destination (%.1f, %.1f, %.1f) at speed %.1f m/s",
          target_point_.x, target_point_.y, target_point_.z, speed_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToDestination::onActionRunning() {
  auto current_pos = cache()->getCurrentPosition();
  double distance = calculateDistance(current_pos, target_point_);

  if (distance < arrival_distance_) {
    logInfo("Reached destination, distance: %.1f m", distance);
    return BT::NodeStatus::SUCCESS;
  }

  // 发送控制指令
  sendOffboardControl();

  // 检查超时
  if (isTimeout()) {
    logError("Go to destination timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void GoToDestination::onActionHalted() {
  logInfo("Go to destination halted");
}

double GoToDestination::calculateDistance(const geometry_msgs::msg::Point& p1,
                                          const geometry_msgs::msg::Point& p2) {
  return std::sqrt(
      std::pow(p1.x - p2.x, 2) +
          std::pow(p1.y - p2.y, 2) +
          std::pow(p1.z - p2.z, 2));
}

void GoToDestination::sendOffboardControl() {
  custom_msgs::msg::OffboardCtrl msg;
  msg.frame = 1; // LOCAL_NED
  msg.type_mask = 0;
  msg.x = target_point_.x;
  msg.y = target_point_.y;
  msg.z = target_point_.z;
  msg.vx = speed_;

  if (use_yaw_) {
    msg.yaw = target_yaw_;
  } else {
    msg.yaw = 0.0f;
  }

  ros()->publishOffboardControl(msg);
}

// ============= SetDestinationPoint 实现 =============

BT::NodeStatus SetDestinationPoint::execute() {
  int step = 1;
  float obs_height = -30.0f;
  std::string pts_param, pt_typ_param;

  getInputValue<int>("step").assign_to(step);
  getInputValue<float>("obsHgh").assign_to(obs_height);
  getInputValue<std::string>("ptsParam").assign_to(pts_param);
  getInputValue<std::string>("ptTypParam").assign_to(pt_typ_param);

  // 获取区域点
  auto area_points_opt = context()->getParameter(pts_param);
  if (!area_points_opt.has_value()) {
    logError("Area points parameter not found: %s", pts_param.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 获取点类型
  PointType point_type = PointType::WAYPOINT;
  auto pt_type_opt = context()->getParameter(pt_typ_param);
  if (pt_type_opt.has_value()) {
    point_type = static_cast<PointType>(pt_type_opt->get<int>());
  }

  try {
    // 从JSON转换为Polygon
    geometry_msgs::msg::Polygon area_polygon;
    // 这里需要JSON到Polygon的转换逻辑

    // 计算下一个目标点
    geometry_msgs::msg::Point target = calculateNextPoint(area_polygon, step, obs_height, point_type);

    // 输出结果
    setOutputValue("target", target);

    logInfo("Set destination point to (%.1f, %.1f, %.1f)", target.x, target.y, target.z);
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    logError("Failed to set destination point: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

geometry_msgs::msg::Point SetDestinationPoint::calculateNextPoint(
    const geometry_msgs::msg::Polygon& area_points,
    int step, float height_offset, PointType point_type) {

  geometry_msgs::msg::Point result;

  if (!area_points.points.empty()) {
    // 简单实现：返回第一个点加上高度偏移
    result.x = area_points.points[0].x;
    result.y = area_points.points[0].y;
    result.z = area_points.points[0].z + height_offset;
  }

  return result;
}

// ============= TrajectoryFollowing 实现 =============

BT::NodeStatus TrajectoryFollowing::onActionStart() {
  if (!getInputValue<std::vector<geometry_msgs::msg::Point>>("trajectory").assign_to(trajectory_)) {
    logError("Failed to get trajectory parameter");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInputValue<float>("speed").assign_to(speed_)) {
    speed_ = 5.0f;
  }

  if (!getInputValue<float>("lookahead_distance").assign_to(lookahead_distance_)) {
    lookahead_distance_ = 10.0f;
  }

  if (trajectory_.empty()) {
    logError("Empty trajectory provided");
    return BT::NodeStatus::FAILURE;
  }

  current_segment_ = 0;

  logInfo("Starting trajectory following with %zu points", trajectory_.size());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrajectoryFollowing::onActionRunning() {
  if (current_segment_ >= trajectory_.size() - 1) {
    logInfo("Trajectory following completed");
    return BT::NodeStatus::SUCCESS;
  }

  // 查找当前最近的轨迹段
  current_segment_ = findClosestSegment();

  // 计算前瞻点
  geometry_msgs::msg::Point lookahead_point = calculateLookaheadPoint();

  // 发送跟踪控制
  sendTrackingControl(lookahead_point);

  // 检查超时
  if (isTimeout()) {
    logError("Trajectory following timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TrajectoryFollowing::onActionHalted() {
  logInfo("Trajectory following halted");
}

geometry_msgs::msg::Point TrajectoryFollowing::calculateLookaheadPoint() {
  // 简单实现：返回前瞻距离内的点
  if (current_segment_ < trajectory_.size()) {
    return trajectory_[current_segment_];
  }
  return geometry_msgs::msg::Point();
}

size_t TrajectoryFollowing::findClosestSegment() {
  auto current_pos = cache()->getCurrentPosition();
  size_t closest_segment = current_segment_;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < trajectory_.size(); ++i) {
    double distance = std::sqrt(
        std::pow(current_pos.x - trajectory_[i].x, 2) +
            std::pow(current_pos.y - trajectory_[i].y, 2) +
            std::pow(current_pos.z - trajectory_[i].z, 2));

    if (distance < min_distance) {
      min_distance = distance;
      closest_segment = i;
    }
  }

  return closest_segment;
}

void TrajectoryFollowing::sendTrackingControl(const geometry_msgs::msg::Point& lookahead_point) {
  custom_msgs::msg::OffboardCtrl msg;
  msg.frame = 1; // LOCAL_NED
  msg.type_mask = 0;
  msg.x = lookahead_point.x;
  msg.y = lookahead_point.y;
  msg.z = lookahead_point.z;
  msg.vx = speed_;
  msg.yaw = 0.0f;

  ros()->publishOffboardControl(msg);
}

// ============= PathPlanning 实现 =============

BT::NodeStatus PathPlanning::onActionStart() {
  if (!getInputValue<geometry_msgs::msg::Point>("start").assign_to(start_point_)) {
    start_point_ = cache()->getCurrentPosition();
  }

  if (!getInputValue<geometry_msgs::msg::Point>("goal").assign_to(goal_point_)) {
    logError("Failed to get goal parameter");
    return BT::NodeStatus::FAILURE;
  }

  getInputValue<geometry_msgs::msg::Polygon>("obstacles").assign_to(obstacles_);

  planning_complete_ = false;

  logInfo("Starting path planning from (%.1f, %.1f, %.1f) to (%.1f, %.1f, %.1f)",
          start_point_.x, start_point_.y, start_point_.z,
          goal_point_.x, goal_point_.y, goal_point_.z);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PathPlanning::onActionRunning() {
  if (!planning_complete_) {
    // 执行路径规划
    planned_path_ = planPath(start_point_, goal_point_, obstacles_);
    planning_complete_ = true;

    if (planned_path_.empty()) {
      logError("Path planning failed - no valid path found");
      return BT::NodeStatus::FAILURE;
    }

    // 输出规划的路径
    setOutputValue("path", planned_path_);

    logInfo("Path planning completed with %zu waypoints", planned_path_.size());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void PathPlanning::onActionHalted() {
  logInfo("Path planning halted");
}

std::vector<geometry_msgs::msg::Point> PathPlanning::planPath(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal,
    const geometry_msgs::msg::Polygon& obstacles) {

  std::vector<geometry_msgs::msg::Point> path;

  // 简单的直线路径规划（实际实现中应该使用A*等算法）
  if (isCollisionFree(start, goal, obstacles)) {
    path.push_back(start);
    path.push_back(goal);
  }

  return path;
}

bool PathPlanning::isCollisionFree(const geometry_msgs::msg::Point& p1,
                                   const geometry_msgs::msg::Point& p2,
                                   const geometry_msgs::msg::Polygon& obstacles) {
  // 简化的碰撞检测
  return true; // 暂时返回true
}

// ============= WaypointMission 实现 =============

BT::NodeStatus WaypointMission::onActionStart() {
  if (!getInputValue<geometry_msgs::msg::Polygon>("waypoints").assign_to(waypoints_)) {
    logError("Failed to get waypoints parameter");
    return BT::NodeStatus::FAILURE;
  }

  getInputValue<std::vector<std::string>>("actions").assign_to(waypoint_actions_);

  if (!getInputValue<float>("speed").assign_to(speed_)) {
    speed_ = 5.0f;
  }

  if (!getInputValue<float>("arrival_distance").assign_to(arrival_distance_)) {
    arrival_distance_ = 3.0f;
  }

  if (!getInputValue<bool>("auto_continue").assign_to(auto_continue_)) {
    auto_continue_ = true;
  }

  if (waypoints_.points.empty()) {
    logError("No waypoints provided");
    return BT::NodeStatus::FAILURE;
  }

  current_waypoint_index_ = 0;
  executing_waypoint_action_ = false;

  logInfo("Starting waypoint mission with %zu waypoints", waypoints_.points.size());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaypointMission::onActionRunning() {
  if (current_waypoint_index_ >= waypoints_.points.size()) {
    logInfo("Waypoint mission completed");
    return BT::NodeStatus::SUCCESS;
  }

  if (!executing_waypoint_action_) {
    // 移动到航点
    moveToWaypoint(current_waypoint_index_);

    // 检查是否到达航点
    auto current_pos = cache()->getCurrentPosition();
    auto& waypoint = waypoints_.points[current_waypoint_index_];

    double distance = std::sqrt(
        std::pow(current_pos.x - waypoint.x, 2) +
            std::pow(current_pos.y - waypoint.y, 2) +
            std::pow(current_pos.z - waypoint.z, 2));

    if (distance < arrival_distance_) {
      // 到达航点，执行相关动作
      if (current_waypoint_index_ < waypoint_actions_.size()) {
        executing_waypoint_action_ = true;
        action_start_time_ = std::chrono::steady_clock::now();
        logInfo("Reached waypoint %zu, executing action: %s",
                current_waypoint_index_, waypoint_actions_[current_waypoint_index_].c_str());
      }
    }
  } else {
    // 执行航点动作
    if (current_waypoint_index_ < waypoint_actions_.size()) {
      if (executeWaypointAction(waypoint_actions_[current_waypoint_index_])) {
        executing_waypoint_action_ = false;
        current_waypoint_index_++;
        logInfo("Waypoint action completed, moving to next waypoint");
      }
    }
  }

  // 检查超时
  if (isTimeout()) {
    logError("Waypoint mission timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void WaypointMission::onActionHalted() {
  logInfo("Waypoint mission halted");
}

bool WaypointMission::executeWaypointAction(const std::string& action) {
  // 执行航点特定动作
  auto elapsed = std::chrono::steady_clock::now() - action_start_time_;

  if (action == "hover") {
    // 悬停3秒
    return elapsed > std::chrono::seconds(3);
  } else if (action == "photo") {
    // 拍照动作
    return elapsed > std::chrono::seconds(1);
  }

  // 默认立即完成
  return true;
}

void WaypointMission::moveToWaypoint(size_t index) {
  if (index < waypoints_.points.size()) {
    auto& waypoint = waypoints_.points[index];

    custom_msgs::msg::OffboardCtrl msg;
    msg.frame = 1; // LOCAL_NED
    msg.type_mask = 0;
    msg.x = waypoint.x;
    msg.y = waypoint.y;
    msg.z = waypoint.z;
    msg.vx = speed_;
    msg.yaw = 0.0f;

    ros()->publishOffboardControl(msg);
  }
}