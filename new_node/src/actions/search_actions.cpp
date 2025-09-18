#include "behavior_node/actions/search_actions.hpp"

#include <cmath>
#include <algorithm>

// ============= SetSearchParameters 实现 =============

BT::NodeStatus SetSearchParameters::execute() {
  std::string area_param, spd_param, tgt_cls_param, pt_typ_param;
  float search_altitude, overlap_ratio;

  getInputValue<std::string>("areaPointsParam").assign_to(area_param);
  getInputValue<std::string>("spdParam").assign_to(spd_param);
  getInputValue<std::string>("tgtClsParam").assign_to(tgt_cls_param);
  getInputValue<std::string>("ptTypParam").assign_to(pt_typ_param);
  getInputValue<float>("search_altitude").assign_to(search_altitude);
  getInputValue<float>("overlap_ratio").assign_to(overlap_ratio);

  logInfo("Setting search parameters");

  try {
    // 设置搜索区域
    auto area_opt = context()->getParameter(area_param);
    if (area_opt.has_value()) {
      // 从JSON转换搜索区域
      context()->setParameter("search_area", area_opt.value());
    }

    // 设置搜索速度
    auto speed_opt = context()->getParameter(spd_param);
    if (speed_opt.has_value()) {
      context()->setSpeed(speed_opt->get<float>());
    }

    // 设置目标类别
    auto tgt_cls_opt = context()->getParameter(tgt_cls_param);
    if (tgt_cls_opt.has_value()) {
      context()->setSearchTargetId(tgt_cls_opt->get<uint8_t>());
    }

    // 设置搜索高度和重叠率
    context()->setParameter("search_altitude", search_altitude);
    context()->setParameter("overlap_ratio", overlap_ratio);

    logInfo("Search parameters set successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    logError("Failed to set search parameters: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ============= PatternSearchAction 实现 =============

BT::NodeStatus PatternSearchAction::onActionStart() {
  if (!getInputValue<geometry_msgs::msg::Polygon>("areaPoints").assign_to(search_area_)) {
    logError("Failed to get search area parameter");
    return BT::NodeStatus::FAILURE;
  }

  std::string pattern_str;
  if (!getInputValue<std::string>("pattern").assign_to(pattern_str)) {
    pattern_str = "GRID";
  }

  // 转换搜索模式
  if (pattern_str == "GRID") search_pattern_ = SearchPattern::GRID;
  else if (pattern_str == "SPIRAL") search_pattern_ = SearchPattern::SPIRAL;
  else if (pattern_str == "ZIGZAG") search_pattern_ = SearchPattern::ZIGZAG;
  else if (pattern_str == "LINE") search_pattern_ = SearchPattern::LINE;
  else search_pattern_ = SearchPattern::GRID;

  if (!getInputValue<float>("spd").assign_to(search_speed_)) {
    search_speed_ = 5.0f;
  }

  if (!getInputValue<float>("altitude").assign_to(search_altitude_)) {
    search_altitude_ = 50.0f;
  }

  getInputValue<TargetClass>("tgtCls").assign_to(target_class_);

  if (!getInputValue<float>("coverage_width").assign_to(coverage_width_)) {
    coverage_width_ = 50.0f;
  }

  if (search_area_.points.empty()) {
    logError("Empty search area provided");
    return BT::NodeStatus::FAILURE;
  }

  // 生成搜索航点
  generateSearchWaypoints();

  if (search_waypoints_.empty()) {
    logError("Failed to generate search waypoints");
    return BT::NodeStatus::FAILURE;
  }

  current_waypoint_index_ = 0;
  search_start_time_ = std::chrono::steady_clock::now();

  logInfo("Starting pattern search with %zu waypoints, pattern: %s",
          search_waypoints_.size(), pattern_str.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PatternSearchAction::onActionRunning() {
  if (isSearchComplete()) {
    logInfo("Pattern search completed");
    return BT::NodeStatus::SUCCESS;
  }

  // 移动到下一个搜索点
  if (!moveToNextSearchPoint()) {
    logError("Failed to move to next search point");
    return BT::NodeStatus::FAILURE;
  }

  // 在当前点执行搜索
  performSearchAtPoint();

  // 检查超时
  if (isTimeout()) {
    logError("Pattern search timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void PatternSearchAction::onActionHalted() {
  logInfo("Pattern search halted");
}

void PatternSearchAction::generateSearchWaypoints() {
  switch (search_pattern_) {
    case SearchPattern::GRID:
      search_waypoints_ = generateGridPattern();
      break;
    case SearchPattern::SPIRAL:
      search_waypoints_ = generateSpiralPattern();
      break;
    case SearchPattern::ZIGZAG:
      search_waypoints_ = generateZigzagPattern();
      break;
    case SearchPattern::LINE:
      search_waypoints_ = generateLinePattern();
      break;
    default:
      search_waypoints_ = generateGridPattern();
      break;
  }
}

std::vector<geometry_msgs::msg::Point> PatternSearchAction::generateGridPattern() {
  std::vector<geometry_msgs::msg::Point> waypoints;

  if (search_area_.points.empty()) return waypoints;

  // 计算搜索区域边界
  float min_x = search_area_.points[0].x, max_x = search_area_.points[0].x;
  float min_y = search_area_.points[0].y, max_y = search_area_.points[0].y;

  for (const auto& point : search_area_.points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  // 生成网格点
  float step = coverage_width_;
  bool reverse = false;

  for (float y = min_y; y <= max_y; y += step) {
    if (reverse) {
      for (float x = max_x; x >= min_x; x -= step) {
        geometry_msgs::msg::Point waypoint;
        waypoint.x = x;
        waypoint.y = y;
        waypoint.z = -search_altitude_; // NED坐标系
        waypoints.push_back(waypoint);
      }
    } else {
      for (float x = min_x; x <= max_x; x += step) {
        geometry_msgs::msg::Point waypoint;
        waypoint.x = x;
        waypoint.y = y;
        waypoint.z = -search_altitude_;
        waypoints.push_back(waypoint);
      }
    }
    reverse = !reverse;
  }

  return waypoints;
}

std::vector<geometry_msgs::msg::Point> PatternSearchAction::generateSpiralPattern() {
  std::vector<geometry_msgs::msg::Point> waypoints;

  // 计算搜索区域中心
  float center_x = 0, center_y = 0;
  for (const auto& point : search_area_.points) {
    center_x += point.x;
    center_y += point.y;
  }
  center_x /= search_area_.points.size();
  center_y /= search_area_.points.size();

  // 生成螺旋模式
  float radius = coverage_width_;
  float angle = 0;

  while (radius < 500.0f) { // 最大半径限制
    float x = center_x + radius * std::cos(angle);
    float y = center_y + radius * std::sin(angle);

    geometry_msgs::msg::Point waypoint;
    waypoint.x = x;
    waypoint.y = y;
    waypoint.z = -search_altitude_;
    waypoints.push_back(waypoint);

    angle += 0.2f; // 角度步长
    radius += 2.0f; // 半径增长
  }

  return waypoints;
}

std::vector<geometry_msgs::msg::Point> PatternSearchAction::generateZigzagPattern() {
  // 简化实现，返回网格模式
  return generateGridPattern();
}

std::vector<geometry_msgs::msg::Point> PatternSearchAction::generateLinePattern() {
  std::vector<geometry_msgs::msg::Point> waypoints;

  // 沿搜索区域边界生成航点
  for (const auto& point : search_area_.points) {
    geometry_msgs::msg::Point waypoint;
    waypoint.x = point.x;
    waypoint.y = point.y;
    waypoint.z = -search_altitude_;
    waypoints.push_back(waypoint);
  }

  return waypoints;
}

bool PatternSearchAction::moveToNextSearchPoint() {
  if (current_waypoint_index_ >= search_waypoints_.size()) {
    return false;
  }

  auto current_pos = cache()->getCurrentPosition();
  auto& target = search_waypoints_[current_waypoint_index_];

  // 检查是否到达当前搜索点
  double distance = std::sqrt(
      std::pow(current_pos.x - target.x, 2) +
          std::pow(current_pos.y - target.y, 2) +
          std::pow(current_pos.z - target.z, 2));

  if (distance < 5.0) { // 5米范围内认为到达
    current_waypoint_index_++;
    logDebug("Reached search point %zu", current_waypoint_index_);
  }

  // 发送控制指令
  if (current_waypoint_index_ < search_waypoints_.size()) {
    custom_msgs::msg::OffboardCtrl msg;
    msg.frame = 1; // LOCAL_NED
    msg.type_mask = 0;
    msg.x = target.x;
    msg.y = target.y;
    msg.z = target.z;
    msg.vx = search_speed_;
    msg.yaw = 0.0f;

    ros()->publishOffboardControl(msg);
  }

  return true;
}

void PatternSearchAction::performSearchAtPoint() {
  // 在当前点执行搜索操作
  // 这里可以添加相机控制、目标检测等逻辑
  logDebug("Performing search at point %zu", current_waypoint_index_);
}

bool PatternSearchAction::isSearchComplete() {
  return current_waypoint_index_ >= search_waypoints_.size();
}

// ============= SearchViaLineAction 实现 =============

BT::NodeStatus SearchViaLineAction::onActionStart() {
  if (!getInputValue<geometry_msgs::msg::Polygon>("areaPoints").assign_to(search_line_)) {
    logError("Failed to get search line parameter");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInputValue<float>("spd").assign_to(search_speed_)) {
    search_speed_ = 5.0f;
  }

  getInputValue<TargetClass>("tgtCls").assign_to(target_class_);

  if (!getInputValue<float>("search_width").assign_to(search_width_)) {
    search_width_ = 100.0f;
  }

  if (!getInputValue<float>("search_height").assign_to(search_height_)) {
    search_height_ = 50.0f;
  }

  if (search_line_.points.empty()) {
    logError("Empty search line provided");
    return BT::NodeStatus::FAILURE;
  }

  current_segment_ = 0;
  segment_start_time_ = std::chrono::steady_clock::now();

  logInfo("Starting search via line with %zu segments", search_line_.points.size() - 1);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchViaLineAction::onActionRunning() {
  if (!hasMoreSegments()) {
    logInfo("Search via line completed");
    return BT::NodeStatus::SUCCESS;
  }

  // 沿当前线段移动
  if (!moveAlongSearchLine()) {
    logError("Failed to move along search line");
    return BT::NodeStatus::FAILURE;
  }

  // 执行搜索
  performSearchAlongSegment();

  // 检查超时
  if (isTimeout()) {
    logError("Search via line timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SearchViaLineAction::onActionHalted() {
  logInfo("Search via line halted");
}

bool SearchViaLineAction::moveAlongSearchLine() {
  if (current_segment_ >= search_line_.points.size() - 1) {
    return false;
  }

  auto current_pos = cache()->getCurrentPosition();
  auto& target = search_line_.points[current_segment_ + 1];

  // 检查是否到达当前线段终点
  double distance = std::sqrt(
      std::pow(current_pos.x - target.x, 2) +
          std::pow(current_pos.y - target.y, 2));

  if (distance < 5.0) { // 5米范围内认为到达
    current_segment_++;
    segment_start_time_ = std::chrono::steady_clock::now();
    logDebug("Completed segment %zu", current_segment_);
  }

  // 发送控制指令
  if (current_segment_ < search_line_.points.size() - 1) {
    custom_msgs::msg::OffboardCtrl msg;
    msg.frame = 1; // LOCAL_NED
    msg.type_mask = 0;
    msg.x = target.x;
    msg.y = target.y;
    msg.z = -search_height_;
    msg.vx = search_speed_;
    msg.yaw = 0.0f;

    ros()->publishOffboardControl(msg);
  }

  return true;
}

void SearchViaLineAction::performSearchAlongSegment() {
  // 沿线段执行搜索操作
  logDebug("Searching along segment %zu", current_segment_);
}

bool SearchViaLineAction::hasMoreSegments() {
  return current_segment_ < search_line_.points.size() - 1;
}

// ============= TargetTrackingAction 实现 =============

BT::NodeStatus TargetTrackingAction::onActionStart() {
  if (!getInputValue<uint8_t>("target_id").assign_to(target_id_)) {
    target_id_ = 0;
  }

  if (!getInputValue<float>("track_distance").assign_to(track_distance_)) {
    track_distance_ = 30.0f;
  }

  if (!getInputValue<float>("track_altitude").assign_to(track_altitude_)) {
    track_altitude_ = 50.0f;
  }

  if (!getInputValue<bool>("continuous_track").assign_to(continuous_track_)) {
    continuous_track_ = true;
  }

  last_update_time_ = std::chrono::steady_clock::now();

  logInfo("Starting target tracking for target ID: %d", target_id_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TargetTrackingAction::onActionRunning() {
  // 更新目标位置
  if (!updateTargetLocation()) {
    if (!continuous_track_) {
      logError("Target lost and continuous tracking disabled");
      return BT::NodeStatus::FAILURE;
    }

    // 使用预测位置继续跟踪
    predicted_position_ = predictTargetPosition();
  }

  // 计算跟踪位置
  geometry_msgs::msg::Point tracking_position;
  if (last_target_location_.header.stamp.sec > 0) {
    geometry_msgs::msg::Point target_pos;
    target_pos.x = last_target_location_.x;
    target_pos.y = last_target_location_.y;
    target_pos.z = last_target_location_.z;
    tracking_position = calculateTrackingPosition(target_pos);
  } else {
    tracking_position = calculateTrackingPosition(predicted_position_);
  }

  // 发送跟踪控制指令
  sendTrackingControl(tracking_position);

  // 检查目标是否可见
  if (!isTargetVisible()) {
    auto elapsed = std::chrono::steady_clock::now() - last_update_time_;
    if (elapsed > std::chrono::seconds(10)) {
      logError("Target lost for too long");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 检查超时
  if (isTimeout()) {
    logError("Target tracking timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TargetTrackingAction::onActionHalted() {
  logInfo("Target tracking halted");
}

bool TargetTrackingAction::updateTargetLocation() {
  auto target_location = cache()->getTargetLocation();
  if (target_location.has_value()) {
    last_target_location_ = target_location.value();
    last_update_time_ = std::chrono::steady_clock::now();
    return true;
  }
  return false;
}

geometry_msgs::msg::Point TargetTrackingAction::predictTargetPosition() {
  // 简单的线性预测
  geometry_msgs::msg::Point predicted;
  predicted.x = last_target_location_.x;
  predicted.y = last_target_location_.y;
  predicted.z = last_target_location_.z;

  // 可以根据目标的历史位置和速度进行更复杂的预测
  return predicted;
}

geometry_msgs::msg::Point TargetTrackingAction::calculateTrackingPosition(
    const geometry_msgs::msg::Point& target_pos) {

  geometry_msgs::msg::Point tracking_pos;

  // 计算相对于目标的跟踪位置
  tracking_pos.x = target_pos.x;
  tracking_pos.y = target_pos.y - track_distance_;
  tracking_pos.z = -track_altitude_;

  return tracking_pos;
}

void TargetTrackingAction::sendTrackingControl(const geometry_msgs::msg::Point& position) {
  custom_msgs::msg::OffboardCtrl msg;
  msg.frame = 1; // LOCAL_NED
  msg.type_mask = 0;
  msg.x = position.x;
  msg.y = position.y;
  msg.z = position.z;
  msg.yaw = 0.0f;

  ros()->publishOffboardControl(msg);
}

bool TargetTrackingAction::isTargetVisible() {
  // 检查目标是否在可见范围内
  auto target_location = cache()->getTargetLocation();
  return target_location.has_value();
}

// ============= TargetConfirmationAction 实现 =============

BT::NodeStatus TargetConfirmationAction::onActionStart() {
  if (!getInputValue<uint8_t>("target_id").assign_to(target_id_)) {
    target_id_ = 0;
  }

  if (!getInputValue<float>("confirmation_distance").assign_to(confirmation_distance_)) {
    confirmation_distance_ = 20.0f;
  }

  if (!getInputValue<float>("confirmation_time").assign_to(confirmation_time_)) {
    confirmation_time_ = 5.0f;
  }

  if (!getInputValue<int>("confirmation_angles").assign_to(confirmation_angles_)) {
    confirmation_angles_ = 3;
  }

  // 获取目标位置
  auto target_location = cache()->getTargetLocation();
  if (!target_location.has_value()) {
    logError("No target location available for confirmation");
    return BT::NodeStatus::FAILURE;
  }

  target_location_ = target_location.value();
  current_confirmation_index_ = 0;
  confirmation_start_time_ = std::chrono::steady_clock::now();

  // 生成确认点
  generateConfirmationPoints();

  logInfo("Starting target confirmation for target ID: %d with %d confirmation points",
          target_id_, confirmation_angles_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TargetConfirmationAction::onActionRunning() {
  if (current_confirmation_index_ >= confirmation_points_.size()) {
    if (isTargetConfirmed()) {
      logInfo("Target confirmation successful");
      return BT::NodeStatus::SUCCESS;
    } else {
      logError("Target confirmation failed");
      return BT::NodeStatus::FAILURE;
    }
  }

  // 移动到确认点
  if (!moveToConfirmationPoint()) {
    logError("Failed to move to confirmation point");
    return BT::NodeStatus::FAILURE;
  }

  // 执行确认
  performConfirmation();

  // 检查超时
  if (isTimeout()) {
    logError("Target confirmation timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TargetConfirmationAction::onActionHalted() {
  logInfo("Target confirmation halted");
}

void TargetConfirmationAction::generateConfirmationPoints() {
  confirmation_points_.clear();

  geometry_msgs::msg::Point target_pos;
  target_pos.x = target_location_.x;
  target_pos.y = target_location_.y;
  target_pos.z = target_location_.z;

  // 在目标周围生成确认点
  for (int i = 0; i < confirmation_angles_; ++i) {
    float angle = 2.0f * M_PI * i / confirmation_angles_;

    geometry_msgs::msg::Point confirmation_point;
    confirmation_point.x = target_pos.x + confirmation_distance_ * std::cos(angle);
    confirmation_point.y = target_pos.y + confirmation_distance_ * std::sin(angle);
    confirmation_point.z = target_pos.z - 30.0f; // 30米高度

    confirmation_points_.push_back(confirmation_point);
  }
}

bool TargetConfirmationAction::moveToConfirmationPoint() {
  if (current_confirmation_index_ >= confirmation_points_.size()) {
    return false;
  }

  auto current_pos = cache()->getCurrentPosition();
  auto& target = confirmation_points_[current_confirmation_index_];

  // 检查是否到达确认点
  double distance = std::sqrt(
      std::pow(current_pos.x - target.x, 2) +
          std::pow(current_pos.y - target.y, 2) +
          std::pow(current_pos.z - target.z, 2));

  if (distance < 3.0) { // 3米范围内认为到达
    current_confirmation_index_++;
    logDebug("Reached confirmation point %zu", current_confirmation_index_);
    return true;
  }

  // 发送控制指令
  custom_msgs::msg::OffboardCtrl msg;
  msg.frame = 1; // LOCAL_NED
  msg.type_mask = 0;
  msg.x = target.x;
  msg.y = target.y;
  msg.z = target.z;
  msg.yaw = 0.0f;

  ros()->publishOffboardControl(msg);

  return true;
}

bool TargetConfirmationAction::performConfirmation() {
  // 在确认点执行确认操作
  logDebug("Performing confirmation at point %zu", current_confirmation_index_);

  // 这里可以添加相机拍照、目标识别等逻辑
  return true;
}

bool TargetConfirmationAction::isTargetConfirmed() {
  // 检查目标是否得到确认
  // 这里应该基于从多个角度的观察结果进行判断
  return true; // 简化实现
}

// ============= AreaScanAction, IntelligentSearchAction 的简化实现 =============

BT::NodeStatus AreaScanAction::onActionStart() {
  logInfo("Area scan action started");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AreaScanAction::onActionRunning() {
  // 简化实现
  return BT::NodeStatus::SUCCESS;
}

void AreaScanAction::onActionHalted() {
  logInfo("Area scan halted");
}

BT::NodeStatus IntelligentSearchAction::onActionStart() {
  logInfo("Intelligent search action started");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus IntelligentSearchAction::onActionRunning() {
  // 简化实现
  return BT::NodeStatus::SUCCESS;
}

void IntelligentSearchAction::onActionHalted() {
  logInfo("Intelligent search halted");
}