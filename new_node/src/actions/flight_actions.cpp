#include "behavior_node/actions/flight_actions.hpp"

#include <chrono>

// ============= LockControl 实现 =============

BT::NodeStatus LockControl::execute() {
  int state;
  if (!getInputValue<int>("state").assign_to(state)) {
    logError("Failed to get state parameter");
    return BT::NodeStatus::FAILURE;
  }

  logInfo("Lock control: %s", state ? "unlock" : "lock");

  try {
    auto future = ros()->callLockControl(state == 1);

    // 等待服务响应
    if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      auto response = future.get();
      if (response && response->success) {
        cache()->setArmed(state == 1);
        logInfo("Lock control successful");
        return BT::NodeStatus::SUCCESS;
      }
    }

    logError("Lock control failed or timeout");
    return BT::NodeStatus::FAILURE;

  } catch (const std::exception& e) {
    logError("Exception in lock control: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ============= FlightModeControl 实现 =============

BT::NodeStatus FlightModeControl::onActionStart() {
  std::string mode_str;
  float param7 = 0.0f;

  if (!getInputValue<std::string>("mode").assign_to(mode_str)) {
    logError("Failed to get mode parameter");
    return BT::NodeStatus::FAILURE;
  }

  getInputValue<float>("param7").assign_to(param7);

  // 转换模式字符串为枚举
  if (mode_str == "MANUAL") target_mode_ = FlightMode::MANUAL;
  else if (mode_str == "ALTITUDE_HOLD") target_mode_ = FlightMode::ALTITUDE_HOLD;
  else if (mode_str == "POSITION_HOLD") target_mode_ = FlightMode::POSITION_HOLD;
  else if (mode_str == "TAKEOFF") target_mode_ = FlightMode::TAKEOFF;
  else if (mode_str == "LAND") target_mode_ = FlightMode::LAND;
  else if (mode_str == "LOITER") target_mode_ = FlightMode::LOITER;
  else if (mode_str == "MISSION") target_mode_ = FlightMode::MISSION;
  else if (mode_str == "RTL") target_mode_ = FlightMode::RTL;
  else if (mode_str == "OFFBOARD") target_mode_ = FlightMode::OFFBOARD;
  else if (mode_str == "STABILIZE") target_mode_ = FlightMode::STABILIZE;
  else {
    logError("Invalid flight mode: %s", mode_str.c_str());
    return BT::NodeStatus::FAILURE;
  }

  logInfo("Setting flight mode to: %s", mode_str.c_str());

  try {
    service_future_ = ros()->callFlightModeControl(static_cast<uint8_t>(target_mode_), param7);
    waiting_for_response_ = true;

    return BT::NodeStatus::RUNNING;

  } catch (const std::exception& e) {
    logError("Exception starting flight mode control: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus FlightModeControl::onActionRunning() {
  if (!waiting_for_response_) {
    // 检查当前飞行模式
    if (cache()->getFlightMode() == target_mode_) {
      logInfo("Flight mode successfully changed");
      return BT::NodeStatus::SUCCESS;
    }

    // 检查超时
    if (isTimeout()) {
      logError("Flight mode change timeout");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  // 检查服务响应
  if (service_future_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
    auto response = service_future_.get();
    waiting_for_response_ = false;

    if (response && response->success) {
      logInfo("Flight mode control command sent successfully");
      return BT::NodeStatus::RUNNING; // 继续等待模式切换完成
    } else {
      logError("Flight mode control service call failed");
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void FlightModeControl::onActionHalted() {
  waiting_for_response_ = false;
  logInfo("Flight mode control halted");
}

// ============= TakeOffAction 实现 =============

BT::NodeStatus TakeOffAction::onActionStart() {
  if (!getInputValue<float>("alt").assign_to(target_altitude_)) {
    target_altitude_ = 10.0f; // 默认高度
  }

  logInfo("Starting takeoff to altitude: %.1f m", target_altitude_);

  // 检查起飞条件
  if (!cache()->hasValidGPS()) {
    logError("GPS not ready for takeoff");
    return BT::NodeStatus::FAILURE;
  }

  if (cache()->getBatteryPercentage() < 25.0f) {
    logError("Battery too low for takeoff: %.1f%%", cache()->getBatteryPercentage());
    return BT::NodeStatus::FAILURE;
  }

  try {
    service_future_ = ros()->callTakeoffControl(target_altitude_);
    service_called_ = true;
    takeoff_start_time_ = std::chrono::steady_clock::now();

    return BT::NodeStatus::RUNNING;

  } catch (const std::exception& e) {
    logError("Exception starting takeoff: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus TakeOffAction::onActionRunning() {
  // 检查服务调用结果
  if (service_called_ &&
      service_future_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
    auto response = service_future_.get();
    service_called_ = false;

    if (!response || !response->success) {
      logError("Takeoff service call failed");
      return BT::NodeStatus::FAILURE;
    }

    logInfo("Takeoff command sent successfully");
  }

  // 检查起飞是否完成
  auto current_pos = cache()->getCurrentPosition();
  float current_altitude = -current_pos.z; // NED坐标系，z向下为正

  if (current_altitude >= target_altitude_ - 1.0f) {
    // 检查是否稳定在目标高度
    auto elapsed = std::chrono::steady_clock::now() - takeoff_start_time_;
    if (elapsed > std::chrono::seconds(3)) {
      logInfo("Takeoff completed successfully at altitude: %.1f m", current_altitude);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // 检查超时
  if (isTimeout()) {
    logError("Takeoff timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void TakeOffAction::onActionHalted() {
  service_called_ = false;
  logInfo("Takeoff action halted");
}

// ============= LandAction 实现 =============

BT::NodeStatus LandAction::onActionStart() {
  if (!getInputValue<float>("descent_rate").assign_to(descent_rate_)) {
    descent_rate_ = 1.0f; // 默认下降速度
  }

  logInfo("Starting landing with descent rate: %.1f m/s", descent_rate_);

  try {
    service_future_ = ros()->callLandControl();
    service_called_ = true;
    land_start_time_ = std::chrono::steady_clock::now();

    return BT::NodeStatus::RUNNING;

  } catch (const std::exception& e) {
    logError("Exception starting landing: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus LandAction::onActionRunning() {
  // 检查服务调用结果
  if (service_called_ &&
      service_future_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
    auto response = service_future_.get();
    service_called_ = false;

    if (!response || !response->success) {
      logError("Landing service call failed");
      return BT::NodeStatus::FAILURE;
    }

    logInfo("Landing command sent successfully");
  }

  // 检查是否已着陆
  if (cache()->getFlightMode() == FlightMode::LAND) {
    auto current_pos = cache()->getCurrentPosition();
    float current_altitude = -current_pos.z;

    // 检查高度是否接近地面
    if (current_altitude < 0.5f) {
      logInfo("Landing completed successfully");
      return BT::NodeStatus::SUCCESS;
    }
  }

  // 检查超时
  if (isTimeout()) {
    logError("Landing timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void LandAction::onActionHalted() {
  service_called_ = false;
  logInfo("Landing action halted");
}

// ============= LoiterAction 实现 =============

BT::NodeStatus LoiterAction::onActionStart() {
  if (!getInputValue<float>("duration").assign_to(duration_)) {
    duration_ = 5.0f; // 默认悬停时间
  }

  // 获取当前位置作为悬停位置
  hover_position_ = cache()->getCurrentPosition();
  loiter_start_time_ = std::chrono::steady_clock::now();

  logInfo("Starting loiter for %.1f seconds at position (%.1f, %.1f, %.1f)",
          duration_, hover_position_.x, hover_position_.y, hover_position_.z);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LoiterAction::onActionRunning() {
  auto elapsed = std::chrono::steady_clock::now() - loiter_start_time_;
  auto elapsed_seconds = std::chrono::duration<float>(elapsed).count();

  if (elapsed_seconds >= duration_) {
    logInfo("Loiter completed");
    return BT::NodeStatus::SUCCESS;
  }

  // 发送悬停控制指令
  custom_msgs::msg::OffboardCtrl ctrl_msg;
  ctrl_msg.frame = 1; // LOCAL_NED
  ctrl_msg.type_mask = 0;
  ctrl_msg.x = hover_position_.x;
  ctrl_msg.y = hover_position_.y;
  ctrl_msg.z = hover_position_.z;
  ctrl_msg.yaw = 0.0f;

  ros()->publishOffboardControl(ctrl_msg);

  return BT::NodeStatus::RUNNING;
}

void LoiterAction::onActionHalted() {
  logInfo("Loiter action halted");
}

// ============= OffboardControl 实现 =============

BT::NodeStatus OffboardControl::execute() {
  geometry_msgs::msg::Point position;
  float yaw = 0.0f;
  int frame = 1;
  int type_mask = 0;

  if (!getInputValue<geometry_msgs::msg::Point>("position").assign_to(position)) {
    logError("Failed to get position parameter");
    return BT::NodeStatus::FAILURE;
  }

  getInputValue<float>("yaw").assign_to(yaw);
  getInputValue<int>("frame").assign_to(frame);
  getInputValue<int>("type_mask").assign_to(type_mask);

  custom_msgs::msg::OffboardCtrl msg = createOffboardMessage(position, yaw, frame, type_mask);
  ros()->publishOffboardControl(msg);

  return BT::NodeStatus::SUCCESS;
}

custom_msgs::msg::OffboardCtrl OffboardControl::createOffboardMessage(
    const geometry_msgs::msg::Point& position, float yaw, int frame, int type_mask) {

  custom_msgs::msg::OffboardCtrl msg;
  msg.frame = frame;
  msg.type_mask = type_mask;
  msg.x = position.x;
  msg.y = position.y;
  msg.z = position.z;
  msg.yaw = yaw;

  return msg;
}

// ============= EmergencyStopAction 实现 =============

BT::NodeStatus EmergencyStopAction::execute() {
  logError("EMERGENCY STOP ACTIVATED");

  // 发送紧急停止指令
  ros()->requestEmergencyStop();

  // 切换到紧急模式
  try {
    auto future = ros()->callFlightModeControl(static_cast<uint8_t>(FlightMode::LOITER));

    // 不等待响应，立即返回成功
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    logError("Exception in emergency stop: %s", e.what());
    return BT::NodeStatus::SUCCESS; // 紧急停止总是返回成功
  }
}

// ============= RTLAction 实现 =============

BT::NodeStatus RTLAction::onActionStart() {
  if (!getInputValue<float>("rtl_altitude").assign_to(rtl_altitude_)) {
    rtl_altitude_ = 50.0f; // 默认返航高度
  }

  home_position_ = cache()->getHomePosition();

  logInfo("Starting RTL to home position (%.1f, %.1f) at altitude %.1f m",
          home_position_.x, home_position_.y, rtl_altitude_);

  try {
    // 使用RTL指令
    auto request = std::make_shared<custom_msgs::srv::CommandLong::Request>();
    request->command = 20; // CMD_NAV_RETURN_TO_LAUNCH
    request->param7 = rtl_altitude_;

    service_future_ = ros()->callService<custom_msgs::srv::CommandLong>(
        "command/long", request);
    service_called_ = true;

    return BT::NodeStatus::RUNNING;

  } catch (const std::exception& e) {
    logError("Exception starting RTL: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus RTLAction::onActionRunning() {
  // 检查服务调用结果
  if (service_called_ &&
      service_future_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
    auto response = service_future_.get();
    service_called_ = false;

    if (!response || !response->success) {
      logError("RTL service call failed");
      return BT::NodeStatus::FAILURE;
    }

    logInfo("RTL command sent successfully");
  }

  // 检查是否到达Home点
  auto current_pos = cache()->getCurrentPosition();
  double distance_to_home = std::sqrt(
      std::pow(current_pos.x - home_position_.x, 2) +
          std::pow(current_pos.y - home_position_.y, 2));

  if (distance_to_home < 5.0) { // 5米范围内认为到达
    logInfo("RTL completed, arrived at home position");
    return BT::NodeStatus::SUCCESS;
  }

  // 检查超时
  if (isTimeout()) {
    logError("RTL timeout");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void RTLAction::onActionHalted() {
  service_called_ = false;
  logInfo("RTL action halted");
}

// ============= SetHomeAction 实现 =============

BT::NodeStatus SetHomeAction::execute() {
  geometry_msgs::msg::Point home_point;
  bool use_current = false;

  getInputValue<bool>("use_current").assign_to(use_current);

  if (use_current) {
    home_point = cache()->getCurrentPosition();
    logInfo("Setting home point to current position: (%.1f, %.1f, %.1f)",
            home_point.x, home_point.y, home_point.z);
  } else {
    if (!getInputValue<geometry_msgs::msg::Point>("home_point").assign_to(home_point)) {
      logError("Failed to get home_point parameter");
      return BT::NodeStatus::FAILURE;
    }
    logInfo("Setting home point to specified position: (%.1f, %.1f, %.1f)",
            home_point.x, home_point.y, home_point.z);
  }

  // 更新缓存
  cache()->setHomePosition(home_point);
  context()->setHomePoint(home_point);

  // 发布home点
  ros()->publishCoordinate(home_point);

  return BT::NodeStatus::SUCCESS;
}

// ============= WeaponControl 实现 =============

BT::NodeStatus WeaponControl::execute() {
  int action;
  if (!getInputValue<int>("action").assign_to(action)) {
    logError("Failed to get action parameter");
    return BT::NodeStatus::FAILURE;
  }

  logInfo("Weapon control: %s", action ? "deploy" : "cancel");

  // 这里可以添加实际的武器控制逻辑
  // 当前只是记录日志

  return BT::NodeStatus::SUCCESS;
}