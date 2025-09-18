#include "behavior_node/core/behavior_executor.hpp"

#include <filesystem>
#include <thread>
#include <chrono>

// 行为节点头文件
#include "behavior_node/actions/flight_actions.hpp"
#include "behavior_node/actions/navigation_actions.hpp"
#include "behavior_node/actions/search_actions.hpp"
#include "behavior_node/actions/control_actions.hpp"
#include "behavior_node/actions/formation_actions.hpp"
#include "behavior_node/conditions/flight_conditions.hpp"
#include "behavior_node/conditions/navigation_conditions.hpp"
#include "behavior_node/conditions/search_conditions.hpp"
#include "behavior_node/conditions/system_conditions.hpp"

BehaviorExecutor::BehaviorExecutor(
    std::shared_ptr<Cache> cache,
    std::shared_ptr<MissionContext> context,
    std::shared_ptr<ROSCommunicationManager> ros_comm,
    std::shared_ptr<behavior_core::MessageQueue> msg_queue,
    const behavior_core::SystemConfig& config)
    : data_cache_(std::move(cache)),
      mission_context_(std::move(context)),
      ros_comm_(std::move(ros_comm)),
      message_queue_(std::move(msg_queue)),
      config_(config),
      node_deps_(ros_comm_, data_cache_, mission_context_, message_queue_) {

  txtLog().info(THISMODULE "BehaviorExecutor created");
}

BehaviorExecutor::~BehaviorExecutor() {
  shutdown();
  txtLog().info(THISMODULE "BehaviorExecutor destroyed");
}

bool BehaviorExecutor::initialize(const std::string& tree_directory) {
  if (is_initialized_.load()) {
    txtLog().warning(THISMODULE "Already initialized");
    return true;
  }

  txtLog().info(THISMODULE "Initializing BehaviorExecutor...");

  try {
    tree_directory_ = tree_directory;

    // 创建黑板
    blackboard_ = BT::Blackboard::create();

    // 设置黑板参数
    setupBlackboard();

    // 注册所有行为节点
    registerAllNodes();

    // 加载主行为树
    if (!loadMainTree()) {
      txtLog().error(THISMODULE "Failed to load main behavior tree");
      return false;
    }

    // 初始化监控
    initializeMonitoring();

    // 更新状态
    updateSystemState(behavior_core::SystemState::IDLE);
    is_initialized_.store(true);
    is_healthy_.store(true);

    txtLog().info(THISMODULE "BehaviorExecutor initialized successfully");
    return true;

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Initialization failed: %s", e.what());
    return false;
  }
}

bool BehaviorExecutor::start() {
  if (!is_initialized_.load()) {
    txtLog().error(THISMODULE "Cannot start - not initialized");
    return false;
  }

  txtLog().info(THISMODULE "Starting BehaviorExecutor...");

  try {
    // 启动消息处理线程
    message_thread_ = std::make_unique<std::thread>(&BehaviorExecutor::messageLoop, this);

    // 启动执行线程
    execution_thread_ = std::make_unique<std::thread>(&BehaviorExecutor::executionLoop, this);

    // 更新状态
    updateSystemState(behavior_core::SystemState::EXECUTING);

    txtLog().info(THISMODULE "BehaviorExecutor started successfully");
    return true;

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Failed to start: %s", e.what());
    return false;
  }
}

void BehaviorExecutor::stop() {
  if (!is_initialized_.load()) {
    return;
  }

  txtLog().info(THISMODULE "Stopping BehaviorExecutor...");

  // 更新状态
  updateSystemState(behavior_core::SystemState::SHUTTING_DOWN);

  // 停止当前树
  if (current_tree_) {
    current_tree_->haltTree();
  }

  // 等待线程结束
  if (execution_thread_ && execution_thread_->joinable()) {
    execution_thread_->join();
  }

  if (message_thread_ && message_thread_->joinable()) {
    message_thread_->join();
  }

  txtLog().info(THISMODULE "BehaviorExecutor stopped");
}

void BehaviorExecutor::shutdown() {
  if (!is_initialized_.load()) {
    return;
  }

  txtLog().info(THISMODULE "Shutting down BehaviorExecutor...");

  stop();
  cleanup();

  is_initialized_.store(false);
  is_healthy_.store(false);

  txtLog().info(THISMODULE "BehaviorExecutor shutdown complete");
}

bool BehaviorExecutor::loadMainTree() {
  try {
    std::string main_tree_path = getTreeFilePath(config_.main_tree_file);

    if (!std::filesystem::exists(main_tree_path)) {
      txtLog().error(THISMODULE "Main tree file not found: %s", main_tree_path.c_str());
      return false;
    }

    // 从文件创建树
    current_tree_ = std::make_unique<BT::Tree>(
        factory_.createTreeFromFile(main_tree_path, blackboard_));

    // 更新执行上下文
    execution_context_.tree_name = "MainBehaviorTree";
    execution_context_.task_name = "main";
    execution_context_.state = behavior_core::TreeState::LOADING;
    execution_context_.start_time = std::chrono::steady_clock::now();
    execution_context_.tick_count = 0;

    txtLog().info(THISMODULE "Main behavior tree loaded successfully");
    return true;

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Failed to load main tree: %s", e.what());
    return false;
  }
}

bool BehaviorExecutor::pause() {
  if (execution_context_.state == behavior_core::TreeState::RUNNING) {
    execution_context_.state = behavior_core::TreeState::PAUSED;
    if (current_tree_) {
      current_tree_->haltTree();
    }
    txtLog().info(THISMODULE "Execution paused");
    return true;
  }
  return false;
}

bool BehaviorExecutor::resume() {
  if (execution_context_.state == behavior_core::TreeState::PAUSED) {
    execution_context_.state = behavior_core::TreeState::RUNNING;
    txtLog().info(THISMODULE "Execution resumed");
    return true;
  }
  return false;
}

void BehaviorExecutor::emergencyStop() {
  txtLog().warning(THISMODULE "EMERGENCY STOP triggered");

  execution_context_.state = behavior_core::TreeState::FAILED;
  updateSystemState(behavior_core::SystemState::ERROR);

  if (current_tree_) {
    current_tree_->haltTree();
  }

  // 发送紧急停止消息到ROS通信管理器
  ros_comm_->requestEmergencyStop();
}

BehaviorExecutor::ExecutionStats BehaviorExecutor::getExecutionStats() const {
  ExecutionStats stats;
  stats.total_ticks = execution_context_.tick_count;
  stats.current_tree_name = execution_context_.tree_name;
  stats.current_state = execution_context_.state;

  if (is_initialized_.load()) {
    auto now = std::chrono::steady_clock::now();
    stats.uptime = std::chrono::duration_cast<std::chrono::seconds>(
        now - execution_context_.start_time);

    if (stats.total_ticks > 0) {
      auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - execution_context_.start_time);
      stats.average_tick_time_ms = static_cast<double>(total_time.count()) / stats.total_ticks;
    }
  }

  return stats;
}

void BehaviorExecutor::registerAllNodes() {
  txtLog().info(THISMODULE "Registering behavior tree nodes...");

  // 注册各类节点
  registerFlightActions();
  registerNavigationActions();
  registerSearchActions();
  registerControlActions();
  registerFormationActions();
  registerConditionNodes();

  txtLog().info(THISMODULE "All nodes registered successfully");
}

void BehaviorExecutor::registerFlightActions() {
  REGISTER_ACTION_NODE(factory_, LockControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, FlightModeControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, TakeOffAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, LandAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, LoiterAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, OffboardControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, EmergencyStopAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, RTLAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, SetHomeAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, WeaponControl, node_deps_);
}

void BehaviorExecutor::registerNavigationActions() {
  REGISTER_ACTION_NODE(factory_, SetLineParameters, node_deps_);
  REGISTER_ACTION_NODE(factory_, SetFormationOffset, node_deps_);
  REGISTER_ACTION_NODE(factory_, NavlineAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, GoToDestination, node_deps_);
  REGISTER_ACTION_NODE(factory_, SetDestinationPoint, node_deps_);
  REGISTER_ACTION_NODE(factory_, TrajectoryFollowing, node_deps_);
  REGISTER_ACTION_NODE(factory_, PathPlanning, node_deps_);
  REGISTER_ACTION_NODE(factory_, WaypointMission, node_deps_);
}

void BehaviorExecutor::registerSearchActions() {
  REGISTER_ACTION_NODE(factory_, SetSearchParameters, node_deps_);
  REGISTER_ACTION_NODE(factory_, PatternSearchAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, SearchViaLineAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, TargetTrackingAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, TargetConfirmationAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, AreaScanAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, IntelligentSearchAction, node_deps_);
}

void BehaviorExecutor::registerControlActions() {
  REGISTER_ACTION_NODE(factory_, TraceAttackControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, NavigationControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, VelocityControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, AttitudeControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, ManualControlOverride, node_deps_);
  REGISTER_ACTION_NODE(factory_, ObstacleAvoidanceControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, EmergencyControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, CameraControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, ImageDistributeControl, node_deps_);
  REGISTER_ACTION_NODE(factory_, SystemSelfCheck, node_deps_);
}

void BehaviorExecutor::registerFormationActions() {
  REGISTER_ACTION_NODE(factory_, SetFormationMembers, node_deps_);
  REGISTER_ACTION_NODE(factory_, FormationSwitch, node_deps_);
  REGISTER_ACTION_NODE(factory_, FormationFlyAction, node_deps_);
  REGISTER_ACTION_NODE(factory_, FormationAssemble, node_deps_);
  REGISTER_ACTION_NODE(factory_, FormationDisband, node_deps_);
  REGISTER_ACTION_NODE(factory_, FormationReorganize, node_deps_);
  REGISTER_ACTION_NODE(factory_, FormationSynchronize, node_deps_);
  REGISTER_ACTION_NODE(factory_, FormationObstacleAvoidance, node_deps_);
}

void BehaviorExecutor::registerConditionNodes() {
  // 飞行条件
  REGISTER_CONDITION_NODE(factory_, CheckArmedCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckFlightModeCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckGPSCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckBatteryCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckTakeoffReadyCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckLandingCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckAltitudeCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckSpeedCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckOffboardReadyCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckEmergencyCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckRTLCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckFlightEnvironmentCondition, node_deps_);

  // 导航条件
  REGISTER_CONDITION_NODE(factory_, CheckArriveDestination, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckWaypointComplete, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckDistanceCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckHomeDistanceCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckPathClearCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckWaypointIndexCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckNavigationStatusCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckInAreaCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckCourseDeviationCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckHeadingCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckNavigationAccuracyCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckGroundTargetDistanceCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckTrajectoryCondition, node_deps_);

  // 搜索条件
  REGISTER_CONDITION_NODE(factory_, CheckQuitSearch, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckTargetDetected, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckTargetLocked, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckTargetTracking, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckSearchAreaComplete, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckTargetVisible, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckSearchPatternCondition, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckMultipleTargets, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckTargetPriority, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckSearchEfficiency, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckSensorStatus, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckSearchTimeLimit, node_deps_);

  // 系统条件
  REGISTER_CONDITION_NODE(factory_, CheckStartTask, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckSystemState, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckCommunicationStatus, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckServiceAvailable, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckMissionTimeout, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckResourceAvailability, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckGroundStationCommand, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckFormationStatus, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckSafetyZone, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckMissionPhase, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckErrorStatus, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckDataValidity, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckManualControl, node_deps_);
  REGISTER_CONDITION_NODE(factory_, CheckMissionComplete, node_deps_);
}

void BehaviorExecutor::setupBlackboard() {
  // 设置默认黑板参数
  blackboard_->set("vehicle_id", data_cache_->getVehicleId());
  blackboard_->set("group_id", data_cache_->getGroupId());
  blackboard_->set("spd", 5.0f);
  blackboard_->set("arvDis", 5.0f);
  blackboard_->set("alt", 10.0f);
  blackboard_->set("loops", 1u);
  blackboard_->set("pointTag", static_cast<int>(PointType::WAYPOINT));
  blackboard_->set("vehiType", static_cast<int>(data_cache_->getVehicleType()));

  txtLog().info(THISMODULE "Blackboard initialized with default parameters");
}

void BehaviorExecutor::initializeMonitoring() {
  if (config_.enable_groot_monitoring) {
    try {
      groot_publisher_ = std::make_unique<BT::Groot2Publisher>(*current_tree_, config_.groot_port);
      txtLog().info(THISMODULE "Groot monitoring enabled on port %d", config_.groot_port);
    } catch (const std::exception& e) {
      txtLog().warning(THISMODULE "Failed to initialize Groot monitoring: %s", e.what());
    }
  }
}

void BehaviorExecutor::executionLoop() {
  txtLog().info(THISMODULE "Starting execution loop");

  execution_context_.state = behavior_core::TreeState::RUNNING;

  auto last_tick = std::chrono::steady_clock::now();
  auto tick_interval = std::chrono::milliseconds(static_cast<int>(1000.0 / config_.tick_rate));

  while (is_initialized_.load() && execution_context_.state != behavior_core::TreeState::FAILED) {
    try {
      auto now = std::chrono::steady_clock::now();

      // 控制tick频率
      if (now - last_tick >= tick_interval) {
        if (execution_context_.state == behavior_core::TreeState::RUNNING && current_tree_) {
          BT::NodeStatus status = executeTick();

          // 根据执行结果更新状态
          if (status == BT::NodeStatus::SUCCESS) {
            updateTreeState(behavior_core::TreeState::SUCCESS);
          } else if (status == BT::NodeStatus::FAILURE) {
            updateTreeState(behavior_core::TreeState::FAILED);
          }

          updateExecutionStats();
        }

        last_tick = now;
      }

      // 短暂休眠
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Exception in execution loop: %s", e.what());
      handleError(std::string("Execution loop error: ") + e.what());
      break;
    }
  }

  txtLog().info(THISMODULE "Execution loop finished");
}

void BehaviorExecutor::messageLoop() {
  txtLog().info(THISMODULE "Starting message processing loop");

  while (is_initialized_.load()) {
    try {
      auto message = message_queue_->waitFor(std::chrono::milliseconds(100));
      if (message.has_value()) {
        processMessage(message.value());
      }

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Exception in message loop: %s", e.what());
    }
  }

  txtLog().info(THISMODULE "Message processing loop finished");
}

BT::NodeStatus BehaviorExecutor::executeTick() {
  if (!current_tree_) {
    return BT::NodeStatus::FAILURE;
  }

  try {
    execution_context_.last_tick = std::chrono::steady_clock::now();
    execution_context_.tick_count++;

    BT::NodeStatus status = current_tree_->tickRoot();

    // 检查是否超过最大tick次数
    if (execution_context_.tick_count >= config_.max_tick_count) {
      txtLog().warning(THISMODULE "Maximum tick count reached");
      return BT::NodeStatus::FAILURE;
    }

    return status;

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Exception during tick: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void BehaviorExecutor::processMessage(const behavior_core::BehaviorMessage& message) {
  switch (message.command) {
    case behavior_core::BehaviorCommand::LOAD_TREE:
      handleLoadTree(message.tree_name, message.parameters);
      break;

    case behavior_core::BehaviorCommand::STOP_TREE:
      handleStopTree();
      break;

    case behavior_core::BehaviorCommand::PAUSE_TREE:
      handlePauseTree();
      break;

    case behavior_core::BehaviorCommand::RESUME_TREE:
      handleResumeTree();
      break;

    case behavior_core::BehaviorCommand::SET_PARAMETER:
      handleSetParameter(message.parameters);
      break;

    case behavior_core::BehaviorCommand::EMERGENCY_STOP:
      emergencyStop();
      break;

    default:
      txtLog().warning(THISMODULE "Unknown behavior command: %d",
                       static_cast<int>(message.command));
      break;
  }
}

void BehaviorExecutor::handleLoadTree(const std::string& tree_name,
                                      const std::unordered_map<std::string, nlohmann::json>& params) {
  txtLog().info(THISMODULE "Loading tree: %s", tree_name.c_str());
  // 主树已经加载，这里可以处理子树切换逻辑
}

void BehaviorExecutor::handleStopTree() {
  txtLog().info(THISMODULE "Stopping tree execution");
  if (current_tree_) {
    current_tree_->haltTree();
  }
  updateTreeState(behavior_core::TreeState::IDLE);
}

void BehaviorExecutor::handlePauseTree() {
  pause();
}

void BehaviorExecutor::handleResumeTree() {
  resume();
}

void BehaviorExecutor::handleSetParameter(const std::unordered_map<std::string, nlohmann::json>& params) {
  for (const auto& [key, value] : params) {
    mission_context_->setParameter(key, value);

    // 同时更新黑板
    if (blackboard_) {
      // 根据参数类型设置黑板值
      if (value.is_string()) {
        blackboard_->set(key, value.get<std::string>());
      } else if (value.is_number_float()) {
        blackboard_->set(key, value.get<float>());
      } else if (value.is_number_integer()) {
        blackboard_->set(key, value.get<int>());
      } else if (value.is_boolean()) {
        blackboard_->set(key, value.get<bool>());
      }
    }
  }

  txtLog().info(THISMODULE "Updated %zu parameters", params.size());
}

void BehaviorExecutor::updateTreeState(behavior_core::TreeState state) {
  execution_context_.state = state;
  txtLog().debug(THISMODULE "Tree state updated to: %d", static_cast<int>(state));
}

void BehaviorExecutor::updateSystemState(behavior_core::SystemState state) {
  mission_context_->setSystemState(state);
  txtLog().debug(THISMODULE "System state updated to: %d", static_cast<int>(state));
}

bool BehaviorExecutor::checkHealth() {
  // 检查基本状态
  if (!is_initialized_.load()) {
    return false;
  }

  // 检查树状态
  if (execution_context_.state == behavior_core::TreeState::FAILED) {
    return false;
  }

  // 检查执行线程
  if (execution_thread_ && !execution_thread_->joinable()) {
    return false;
  }

  // 检查消息处理线程
  if (message_thread_ && !message_thread_->joinable()) {
    return false;
  }

  return true;
}

void BehaviorExecutor::handleError(const std::string& error_msg) {
  txtLog().error(THISMODULE "Handling error: %s", error_msg.c_str());

  execution_context_.last_error = error_msg;
  is_healthy_.store(false);

  // 根据错误严重程度决定处理方式
  if (error_msg.find("critical") != std::string::npos) {
    emergencyStop();
  }
}

std::string BehaviorExecutor::getTreeFilePath(const std::string& tree_name) const {
  return tree_directory_ + "/" + tree_name;
}

bool BehaviorExecutor::isValidTreeName(const std::string& tree_name) const {
  if (tree_name.empty()) {
    return false;
  }

  // 检查文件是否存在
  std::string tree_path = getTreeFilePath(tree_name);
  return std::filesystem::exists(tree_path);
}

void BehaviorExecutor::cleanup() {
  if (current_tree_) {
    current_tree_->haltTree();
    current_tree_.reset();
  }

  if (groot_publisher_) {
    groot_publisher_.reset();
  }

  execution_thread_.reset();
  message_thread_.reset();
}

void BehaviorExecutor::updateExecutionStats() {
  // 这里可以添加更多统计信息的更新
}
