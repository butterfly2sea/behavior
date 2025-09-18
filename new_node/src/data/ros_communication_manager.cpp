#include "behavior_node/data/ros_communication_manager.hpp"

ROSCommunicationManager::ROSCommunicationManager(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<Cache> cache,
    std::shared_ptr<MissionContext> context,
    std::shared_ptr<behavior_core::MessageQueue> msg_queue)
    : node_(std::move(node)),
      data_cache_(std::move(cache)),
      mission_context_(std::move(context)),
      message_queue_(std::move(msg_queue)),
      logger_(node_->get_logger()) {

  txtLog().info(THISMODULE "ROSCommunicationManager created");
}

void ROSCommunicationManager::initialize() {
  txtLog().info(THISMODULE "Initializing ROS communication interfaces...");

  setupPublishers();
  setupSubscriptions();
  setupServiceClients();

  txtLog().info(THISMODULE "ROS communication manager initialized successfully");
}

void ROSCommunicationManager::setupPublishers() {
  try {
    using namespace ros_interface;

    // 图像分发发布
    publishers_[topics::INFO_IMAGE_DISTRIBUTE] =
        node_->create_publisher<custom_msgs::msg::ImageDistribute>(
            topics::INFO_IMAGE_DISTRIBUTE, rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE));

    // home点发布
    publishers_[topics::SET_COORD] =
        node_->create_publisher<geometry_msgs::msg::Point>(
            topics::SET_COORD, rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE));

    // offboard控制发布
    publishers_[topics::OFFBOARD_CONTROL] =
        node_->create_publisher<custom_msgs::msg::OffboardCtrl>(
            topics::OFFBOARD_CONTROL, rclcpp::SensorDataQoS());

    // 飞机类型设置发布
    publishers_[topics::SET_VEHICLE_TYPE] =
        node_->create_publisher<std_msgs::msg::UInt8>(
            topics::SET_VEHICLE_TYPE, rclcpp::QoS(qos::CONTROL_QUEUE_SIZE));

    // 外部响应发布
    publishers_[topics::OUTER_RESPONSE] =
        node_->create_publisher<custom_msgs::msg::CommandResponse>(
            topics::OUTER_RESPONSE, rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE));

    // 外部状态发布
    publishers_[topics::OUTER_STATUS_TASK] =
        node_->create_publisher<custom_msgs::msg::StatusTask>(
            topics::OUTER_STATUS_TASK, rclcpp::QoS(qos::STATUS_QUEUE_SIZE));

    // 防撞距离发布
    publishers_[topics::SET_ANTI_COLLISION_DIS] =
        node_->create_publisher<std_msgs::msg::Float32>(
            topics::SET_ANTI_COLLISION_DIS, rclcpp::QoS(qos::CONTROL_QUEUE_SIZE));

    // 航线发布
    publishers_[topics::SET_NAVLINE] =
        node_->create_publisher<geometry_msgs::msg::Polygon>(
            topics::SET_NAVLINE, rclcpp::QoS(qos::CONTROL_QUEUE_SIZE));

    // 编队成员发布
    publishers_[topics::SET_GROUP_IDS] =
        node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
            topics::SET_GROUP_IDS, rclcpp::QoS(qos::CONTROL_QUEUE_SIZE));

    // 编队速度发布
    publishers_[topics::SET_GROUP_SPEED] =
        node_->create_publisher<std_msgs::msg::Float32>(
            topics::SET_GROUP_SPEED, rclcpp::QoS(qos::CONTROL_QUEUE_SIZE));

    // 编队偏移发布
    publishers_[topics::SET_GROUP_OFFSET] =
        node_->create_publisher<geometry_msgs::msg::Polygon>(
            topics::SET_GROUP_OFFSET, rclcpp::QoS(qos::CONTROL_QUEUE_SIZE));

    // 编队循环次数发布
    publishers_[topics::SET_GROUP_LOOPS] =
        node_->create_publisher<std_msgs::msg::UInt32>(
            topics::SET_GROUP_LOOPS, rclcpp::QoS(qos::CONTROL_QUEUE_SIZE));

    // 点类型发布
    publishers_[topics::SET_POINT_TAG] =
        node_->create_publisher<std_msgs::msg::UInt8>(
            topics::SET_POINT_TAG, rclcpp::QoS(qos::CONTROL_QUEUE_SIZE));

    // 地面站状态发布
    publishers_[topics::GROUND_STATION_STATUS] =
        node_->create_publisher<std_msgs::msg::String>(
            topics::GROUND_STATION_STATUS, rclcpp::QoS(qos::STATUS_QUEUE_SIZE));

    txtLog().info(THISMODULE "All publishers created successfully");

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Failed to create publishers: %s", e.what());
    throw;
  }
}

void ROSCommunicationManager::setupSubscriptions() {
  try {
    using namespace ros_interface;

    // 外部指令订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::CommandRequest>(
            topics::OUTER_COMMAND_REQUEST,
            rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE),
            std::bind(&ROSCommunicationManager::handleCommandRequest, this, std::placeholders::_1)));

    // 地面站指令订阅
    subscriptions_.push_back(
        node_->create_subscription<std_msgs::msg::String>(
            topics::GROUND_STATION_COMMAND,
            rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE),
            std::bind(&ROSCommunicationManager::handleGroundStationCommand, this, std::placeholders::_1)));

    // 简单飞行器信息订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::SimpleVehicle>(
            topics::INFO_SIMPLE_VEHICLE,
            rclcpp::SensorDataQoS(),
            std::bind(&ROSCommunicationManager::handleSimpleVehicle, this, std::placeholders::_1)));

    // 其他飞行器信息订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::SimpleVehicle>(
            topics::OUTER_INFORMATION_SIMPLE_VEHICLE,
            rclcpp::SensorDataQoS(),
            std::bind(&ROSCommunicationManager::handleOtherVehicleInfo, this, std::placeholders::_1)));

    // 目标计算信息订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::ObjectComputation>(
            topics::OUTER_INFORMATION_OBJECT_COMPUTATION,
            rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE),
            std::bind(&ROSCommunicationManager::handleObjectComputation, this, std::placeholders::_1)));

    // 目标位置信息订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::ObjectLocation>(
            topics::OUTER_INFORMATION_OBJECT_LOCATION,
            rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE),
            std::bind(&ROSCommunicationManager::handleObjectLocation, this, std::placeholders::_1)));

    // 距离目标信息订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::DisTarget>(
            topics::INFO_DIS_TARGET,
            rclcpp::SensorDataQoS(),
            std::bind(&ROSCommunicationManager::handleDisTarget, this, std::placeholders::_1)));

    // 任务阶段信息订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::TaskStage>(
            topics::INFO_TASK_STAGE,
            rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE),
            std::bind(&ROSCommunicationManager::handleTaskStage, this, std::placeholders::_1)));

    // 遥控器输入订阅
    subscriptions_.push_back(
        node_->create_subscription<sensor_msgs::msg::Joy>(
            topics::INFO_JOYSTICK,
            rclcpp::SensorDataQoS(),
            std::bind(&ROSCommunicationManager::handleJoystick, this, std::placeholders::_1)));

    // 手动控制订阅
    subscriptions_.push_back(
        node_->create_subscription<mavros_msgs::msg::ManualControl>(
            topics::INFO_MANUAL_CONTROL,
            rclcpp::SensorDataQoS(),
            std::bind(&ROSCommunicationManager::handleManualControl, this, std::placeholders::_1)));

    // 攻击指定目标订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::ObjectAttackDesignate>(
            topics::INFO_TARGET_ATTACK_DESIGNATE,
            rclcpp::QoS(qos::RELIABLE_QUEUE_SIZE),
            std::bind(&ROSCommunicationManager::handleAttackDesignate, this, std::placeholders::_1)));

    // 相机控制订阅
    subscriptions_.push_back(
        node_->create_subscription<custom_msgs::msg::MultispectralCamCtrl>(
            topics::INFO_MULTISPECTRAL_CAM_CTRL,
            rclcpp::QoS(qos::CONTROL_QUEUE_SIZE),
            std::bind(&ROSCommunicationManager::handleCameraControl, this, std::placeholders::_1)));

    txtLog().info(THISMODULE "All subscriptions created successfully");

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Failed to create subscriptions: %s", e.what());
    throw;
  }
}

void ROSCommunicationManager::setupServiceClients() {
  try {
    using namespace ros_interface;

    // 基础控制服务
    service_clients_[services::COMMAND_BOOL] =
        node_->create_client<custom_msgs::srv::CommandBool>(services::COMMAND_BOOL);

    service_clients_[services::COMMAND_INT] =
        node_->create_client<custom_msgs::srv::CommandInt>(services::COMMAND_INT);

    service_clients_[services::COMMAND_LONG] =
        node_->create_client<custom_msgs::srv::CommandLong>(services::COMMAND_LONG);

    service_clients_[services::COMMAND_STRING] =
        node_->create_client<custom_msgs::srv::CommandString>(services::COMMAND_STRING);

    // 专用服务
    service_clients_[services::LOCK_CONTROL] =
        node_->create_client<custom_msgs::srv::CommandBool>(services::LOCK_CONTROL);

    service_clients_[services::FLIGHT_MODE_CONTROL] =
        node_->create_client<custom_msgs::srv::CommandLong>(services::FLIGHT_MODE_CONTROL);

    service_clients_[services::TAKEOFF_CONTROL] =
        node_->create_client<custom_msgs::srv::CommandLong>(services::TAKEOFF_CONTROL);

    service_clients_[services::LAND_CONTROL] =
        node_->create_client<custom_msgs::srv::CommandLong>(services::LAND_CONTROL);

    service_clients_[services::NAVIGATION_CONTROL] =
        node_->create_client<custom_msgs::srv::CommandString>(services::NAVIGATION_CONTROL);

    service_clients_[services::TRACE_ATTACK_CONTROL] =
        node_->create_client<custom_msgs::srv::CommandLong>(services::TRACE_ATTACK_CONTROL);

    service_clients_[services::FORMATION_SWITCH] =
        node_->create_client<custom_msgs::srv::CommandInt>(services::FORMATION_SWITCH);

    txtLog().info(THISMODULE "All service clients created successfully");

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Failed to create service clients: %s", e.what());
    throw;
  }
}

// 发布接口实现
void ROSCommunicationManager::publishOffboardControl(const custom_msgs::msg::OffboardCtrl& msg) {
  publish(ros_interface::topics::OFFBOARD_CONTROL, msg);
}

void ROSCommunicationManager::publishCommandResponse(const custom_msgs::msg::CommandResponse& msg) {
  publish(ros_interface::topics::OUTER_RESPONSE, msg);
}

void ROSCommunicationManager::publishTaskStatus(const custom_msgs::msg::StatusTask& msg) {
  publish(ros_interface::topics::OUTER_STATUS_TASK, msg);
}

void ROSCommunicationManager::publishImageDistribute(const custom_msgs::msg::ImageDistribute& msg) {
  publish(ros_interface::topics::INFO_IMAGE_DISTRIBUTE, msg);
}

void ROSCommunicationManager::publishCoordinate(const geometry_msgs::msg::Point& point) {
  publish(ros_interface::topics::SET_COORD, point);
}

void ROSCommunicationManager::publishVehicleType(uint8_t type) {
  std_msgs::msg::UInt8 msg;
  msg.data = type;
  publish(ros_interface::topics::SET_VEHICLE_TYPE, msg);
}

void ROSCommunicationManager::publishAntiCollisionDistance(float distance) {
  std_msgs::msg::Float32 msg;
  msg.data = distance;
  publish(ros_interface::topics::SET_ANTI_COLLISION_DIS, msg);
}

void ROSCommunicationManager::publishNavline(const geometry_msgs::msg::Polygon& waypoints) {
  publish(ros_interface::topics::SET_NAVLINE, waypoints);
}

void ROSCommunicationManager::publishGroupIds(const std::vector<uint8_t>& ids) {
  std_msgs::msg::UInt8MultiArray msg;
  msg.data = ids;
  publish(ros_interface::topics::SET_GROUP_IDS, msg);
}

void ROSCommunicationManager::publishGroupSpeed(float speed) {
  std_msgs::msg::Float32 msg;
  msg.data = speed;
  publish(ros_interface::topics::SET_GROUP_SPEED, msg);
}

void ROSCommunicationManager::publishGroupOffset(const geometry_msgs::msg::Polygon& offsets) {
  publish(ros_interface::topics::SET_GROUP_OFFSET, offsets);
}

void ROSCommunicationManager::publishGroupLoops(uint32_t loops) {
  std_msgs::msg::UInt32 msg;
  msg.data = loops;
  publish(ros_interface::topics::SET_GROUP_LOOPS, msg);
}

void ROSCommunicationManager::publishPointTag(uint8_t tag) {
  std_msgs::msg::UInt8 msg;
  msg.data = tag;
  publish(ros_interface::topics::SET_POINT_TAG, msg);
}

void ROSCommunicationManager::publishGroundStationStatus(const std::string& status) {
  std_msgs::msg::String msg;
  msg.data = status;
  publish(ros_interface::topics::GROUND_STATION_STATUS, msg);
}

// 服务调用接口实现
bool ROSCommunicationManager::isServiceReady(const std::string& service_name) {
  if (service_clients_.contains(service_name)) {
    if (auto client = std::dynamic_pointer_cast<rclcpp::ClientBase>(
        service_clients_.at(service_name))) {
      return client->wait_for_service(std::chrono::milliseconds(100)) &&
          client->service_is_ready();
    }
  }
  return false;
}

std::future<custom_msgs::srv::CommandBool::Response::SharedPtr>
ROSCommunicationManager::callLockControl(bool lock) {
  auto request = std::make_shared<custom_msgs::srv::CommandBool::Request>();
  request->value = lock;
  return callService<custom_msgs::srv::CommandBool>(
      ros_interface::services::LOCK_CONTROL, request);
}

std::future<custom_msgs::srv::CommandLong::Response::SharedPtr>
ROSCommunicationManager::callFlightModeControl(uint8_t mode, float param7) {
  auto request = std::make_shared<custom_msgs::srv::CommandLong::Request>();
  request->command = ros_interface::commands::CMD_FLIGHT_MODE;
  request->param1 = mode;
  request->param7 = param7;
  return callService<custom_msgs::srv::CommandLong>(
      ros_interface::services::FLIGHT_MODE_CONTROL, request);
}

std::future<custom_msgs::srv::CommandLong::Response::SharedPtr>
ROSCommunicationManager::callTakeoffControl(float altitude) {
  auto request = std::make_shared<custom_msgs::srv::CommandLong::Request>();
  request->command = ros_interface::commands::CMD_TAKEOFF;
  request->param7 = altitude;
  return callService<custom_msgs::srv::CommandLong>(
      ros_interface::services::TAKEOFF_CONTROL, request);
}

std::future<custom_msgs::srv::CommandLong::Response::SharedPtr>
ROSCommunicationManager::callLandControl() {
  auto request = std::make_shared<custom_msgs::srv::CommandLong::Request>();
  request->command = ros_interface::commands::CMD_LAND;
  return callService<custom_msgs::srv::CommandLong>(
      ros_interface::services::LAND_CONTROL, request);
}

std::future<custom_msgs::srv::CommandString::Response::SharedPtr>
ROSCommunicationManager::callNavigationControl(const std::string& command) {
  auto request = std::make_shared<custom_msgs::srv::CommandString::Request>();
  request->value = command;
  return callService<custom_msgs::srv::CommandString>(
      ros_interface::services::NAVIGATION_CONTROL, request);
}

std::future<custom_msgs::srv::CommandLong::Response::SharedPtr>
ROSCommunicationManager::callTraceAttackControl(uint8_t frame, uint8_t command) {
  auto request = std::make_shared<custom_msgs::srv::CommandLong::Request>();
  request->param1 = frame;
  request->param2 = command;
  return callService<custom_msgs::srv::CommandLong>(
      ros_interface::services::TRACE_ATTACK_CONTROL, request);
}

std::future<custom_msgs::srv::CommandInt::Response::SharedPtr>
ROSCommunicationManager::callFormationSwitch(int formation_type) {
  auto request = std::make_shared<custom_msgs::srv::CommandInt::Request>();
  request->value = formation_type;
  return callService<custom_msgs::srv::CommandInt>(
      ros_interface::services::FORMATION_SWITCH, request);
}

// 行为树控制接口
void ROSCommunicationManager::requestTreeLoad(const std::string& tree_name) {
  if (message_queue_) {
    behavior_core::BehaviorMessage msg(behavior_core::BehaviorCommand::LOAD_TREE, tree_name, {});
    message_queue_->push(msg);
    txtLog().info(THISMODULE "Requested to load tree: %s", tree_name.c_str());
  }
}

void ROSCommunicationManager::requestTreeStop() {
  if (message_queue_) {
    behavior_core::BehaviorMessage msg(behavior_core::BehaviorCommand::STOP_TREE, "", {});
    message_queue_->push(msg);
    txtLog().info(THISMODULE "Requested to stop tree");
  }
}

void ROSCommunicationManager::requestTreePause() {
  if (message_queue_) {
    behavior_core::BehaviorMessage msg(behavior_core::BehaviorCommand::PAUSE_TREE, "", {});
    message_queue_->push(msg);
    txtLog().info(THISMODULE "Requested to pause tree");
  }
}

void ROSCommunicationManager::requestTreeResume() {
  if (message_queue_) {
    behavior_core::BehaviorMessage msg(behavior_core::BehaviorCommand::RESUME_TREE, "", {});
    message_queue_->push(msg);
    txtLog().info(THISMODULE "Requested to resume tree");
  }
}

void ROSCommunicationManager::requestEmergencyStop() {
  if (message_queue_) {
    behavior_core::BehaviorMessage msg(behavior_core::BehaviorCommand::EMERGENCY_STOP, "", {});
    message_queue_->push(msg);
    txtLog().warning(THISMODULE "EMERGENCY STOP requested");
  }
}

// 消息处理方法
void ROSCommunicationManager::handleCommandRequest(
    const custom_msgs::msg::CommandRequest::SharedPtr msg) {

  txtLog().info(THISMODULE "Received command request: type=%d, dst=%d",
      msg->type, msg->dst);

  // 检查是否针对本机
  if (!isVehicleTargeted(msg->dst, msg->grp)) {
    return;
  }

  // 处理不同类型的指令
  switch (msg->type) {
    case ros_interface::commands::CMD_TAKEOFF:
      processGroundStationCommand({
                                      .cmd = "start",
                                      .action = "TakeOff",
                                      .vehicle_id = msg->dst,
                                      .group_id = msg->grp,
                                      .params = {{"alt", nlohmann::json(msg->fparam7)}}
                                  });
      break;

    case ros_interface::commands::CMD_LAND:
      processGroundStationCommand({
                                      .cmd = "start",
                                      .action = "Land",
                                      .vehicle_id = msg->dst,
                                      .group_id = msg->grp
                                  });
      break;

    case ros_interface::commands::CMD_MISSION_START:
      // 根据param0决定具体任务
      if (msg->param0 == 1) { // 导航任务
        processGroundStationCommand({
                                        .cmd = "start",
                                        .action = "Navline",
                                        .vehicle_id = msg->dst,
                                        .group_id = msg->grp
                                    });
      }
      break;

    default:
      txtLog().warning(THISMODULE "Unknown command type: %d", msg->type);
      break;
  }
}

void ROSCommunicationManager::handleGroundStationCommand(
    const std_msgs::msg::String::SharedPtr msg) {

  try {
    GroundStationCommand cmd = parseGroundStationCommand(msg->data);

    if (!validateGroundStationCommand(cmd)) {
      txtLog().error(THISMODULE "Invalid ground station command received");
      sendCommandResponse("failure", "Invalid command format");
      return;
    }

    txtLog().info(THISMODULE "Received ground station command: %s %s",
        cmd.cmd.c_str(), cmd.action.c_str());

    processGroundStationCommand(cmd);

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Failed to process ground station command: %s", e.what());
    sendCommandResponse("failure", std::string("Processing error: ") + e.what());
  }
}

void ROSCommunicationManager::handleSimpleVehicle(
    const custom_msgs::msg::SimpleVehicle::SharedPtr msg) {
  // 更新本机状态
  data_cache_->updateSimpleVehicle(*msg);
}

void ROSCommunicationManager::handleOtherVehicleInfo(
    const custom_msgs::msg::SimpleVehicle::SharedPtr msg) {
  // 更新其他飞行器信息
  data_cache_->updateOtherVehicle(msg->id, *msg);
}

void ROSCommunicationManager::handleObjectComputation(
    const custom_msgs::msg::ObjectComputation::SharedPtr msg) {
  // 添加检测到的目标
  data_cache_->addDetectedObject(*msg);
}

void ROSCommunicationManager::handleObjectLocation(
    const custom_msgs::msg::ObjectLocation::SharedPtr msg) {
  // 更新目标位置
  data_cache_->setTargetLocation(*msg);
}

void ROSCommunicationManager::handleDisTarget(
    const custom_msgs::msg::DisTarget::SharedPtr msg) {
  // 更新目标距离信息
  data_cache_->setCurrentTargetInfo(*msg);
}

void ROSCommunicationManager::handleTaskStage(
    const custom_msgs::msg::TaskStage::SharedPtr msg) {
  // 更新任务阶段信息
  data_cache_->setCurrentTaskStage(*msg);
}

void ROSCommunicationManager::handleJoystick(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  // 更新遥控器数据
  data_cache_->setJoystickData(*msg);
}

void ROSCommunicationManager::handleManualControl(
    const mavros_msgs::msg::ManualControl::SharedPtr msg) {
  // 更新手动控制数据
  data_cache_->setManualControl(*msg);
}

void ROSCommunicationManager::handleAttackDesignate(
    const custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg) {
  // 更新攻击指定目标
  data_cache_->setAttackDesignate(*msg);
}

void ROSCommunicationManager::handleCameraControl(
    const custom_msgs::msg::MultispectralCamCtrl::SharedPtr msg) {
  // 更新相机控制数据
  data_cache_->setCameraControl(*msg);
}

// 辅助方法实现
GroundStationCommand ROSCommunicationManager::parseGroundStationCommand(
    const std::string& json_str) {

  GroundStationCommand cmd;

  try {
    nlohmann::json json_data = nlohmann::json::parse(json_str);

    cmd.cmd = json_data.value("cmd", "");
    cmd.action = json_data.value("action", "");
    cmd.vehicle_id = json_data.value("vehicle_id", static_cast<uint8_t>(0xFF));
    cmd.group_id = json_data.value("group_id", static_cast<uint8_t>(0));
    cmd.timestamp = json_data.value("timestamp", "");

    if (json_data.contains("params") && json_data["params"].is_object()) {
      cmd.params = json_data["params"];
    }

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Failed to parse ground station command JSON: %s", e.what());
  }

  return cmd;
}

bool ROSCommunicationManager::validateGroundStationCommand(const GroundStationCommand& cmd) {
  // 验证指令基本格式
  if (cmd.cmd.empty() || cmd.action.empty()) {
    return false;
  }

  // 验证指令类型
  std::vector<std::string> valid_commands = {"start", "stop", "pause", "resume", "set", "emergency"};
  if (std::find(valid_commands.begin(), valid_commands.end(), cmd.cmd) == valid_commands.end()) {
    return false;
  }

  // 验证动作类型
  std::vector<std::string> valid_actions = {
      "TakeOff", "Land", "Navline", "PatternSearch", "SrchViaLine",
      "FormFly", "AutoTrace", "Hover", "Emergency"
  };
  if (std::find(valid_actions.begin(), valid_actions.end(), cmd.action) == valid_actions.end()) {
    return false;
  }

  return true;
}

void ROSCommunicationManager::processGroundStationCommand(const GroundStationCommand& cmd) {
  // 检查是否针对本机
  if (!isVehicleTargeted(cmd.vehicle_id, cmd.group_id)) {
    return;
  }

  // 将参数存储到任务上下文
  for (const auto& [key, value] : cmd.params) {
    mission_context_->setParameter(key, value);
  }

  // 更新地面站指令
  mission_context_->setGroundStationCommand(cmd.action);

  // 根据指令类型创建行为消息
  behavior_core::BehaviorCommand behavior_cmd = behavior_core::BehaviorCommand::NONE;

  if (cmd.cmd == "start") {
    behavior_cmd = behavior_core::BehaviorCommand::LOAD_TREE;
  } else if (cmd.cmd == "stop") {
    behavior_cmd = behavior_core::BehaviorCommand::STOP_TREE;
  } else if (cmd.cmd == "pause") {
    behavior_cmd = behavior_core::BehaviorCommand::PAUSE_TREE;
  } else if (cmd.cmd == "resume") {
    behavior_cmd = behavior_core::BehaviorCommand::RESUME_TREE;
  } else if (cmd.cmd == "emergency") {
    behavior_cmd = behavior_core::BehaviorCommand::EMERGENCY_STOP;
  } else if (cmd.cmd == "set") {
    behavior_cmd = behavior_core::BehaviorCommand::SET_PARAMETER;
  }

  // 发送行为消息
  if (behavior_cmd != behavior_core::BehaviorCommand::NONE && message_queue_) {
    behavior_core::BehaviorMessage msg(behavior_cmd, cmd.action, cmd.params);
    message_queue_->push(msg);
    txtLog().info(THISMODULE "Queued behavior command: %s %s",
        cmd.cmd.c_str(), cmd.action.c_str());
  }

  // 发送成功响应
  sendCommandResponse("success", "Command processed successfully");
}

void ROSCommunicationManager::sendCommandResponse(const std::string& result,
                                                  const std::string& message) {
  custom_msgs::msg::CommandResponse response;
  response.rslt = result;
  response.id = data_cache_->getVehicleId();
  response.msg = message;
  publishCommandResponse(response);
}

void ROSCommunicationManager::sendTaskStatusUpdate(StatusStage status,
                                                   const std::string& description) {
  custom_msgs::msg::StatusTask status_msg;
  status_msg.stage = static_cast<uint8_t>(status);
  status_msg.id = data_cache_->getVehicleId();
  status_msg.description = description;
  publishTaskStatus(status_msg);
}

void ROSCommunicationManager::handleError(const std::string& operation,
                                          const std::exception& e) {
  txtLog().error(THISMODULE "Error in %s: %s", operation.c_str(), e.what());
  sendCommandResponse("failure", std::string(operation) + " failed: " + e.what());
}

bool ROSCommunicationManager::isCommandAllowed(const std::string& cmd) const {
  // 可以在这里添加权限检查逻辑
  return true;
}

bool ROSCommunicationManager::isVehicleTargeted(uint8_t vehicle_id, uint8_t group_id) const {
  uint8_t my_vehicle_id = data_cache_->getVehicleId();
  uint8_t my_group_id = data_cache_->getGroupId();

  // 广播消息 (0xFF)
  if (vehicle_id == 0xFF) {
    return true;
  }

  // 直接指定本机
  if (vehicle_id == my_vehicle_id) {
    return true;
  }

  // 指定分组且本机在该分组中
  if (group_id != 0 && group_id == my_group_id) {
    return true;
  }

  return false;
}