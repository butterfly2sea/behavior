#pragma once

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <memory>
#include <chrono>
#include <thread>
#include <log/Logger.hpp>

#include "behavior_node/core/behavior_executor.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/core/message_queue.hpp"
#include "behavior_node/core/types.hpp"

class BehaviorControlNode : public rclcpp::Node {
 private:
  // 核心组件
  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;
  std::shared_ptr<behavior_core::MessageQueue> message_queue_;
  std::shared_ptr<BehaviorExecutor> behavior_executor_;
  std::shared_ptr<ROSCommunicationManager> ros_comm_;

  // 状态管理
  std::atomic<bool> initialized_{false};
  std::atomic<bool> running_{false};

  // 定时器 - 这是BehaviorControlNode的主要职责
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr message_process_timer_;
  rclcpp::TimerBase::SharedPtr tree_tick_timer_;

  // 配置参数
  std::string tree_directory_;
  std::chrono::milliseconds status_interval_{};
  std::chrono::milliseconds watchdog_interval_{};
  std::chrono::milliseconds message_process_interval_{};
  std::chrono::milliseconds tree_tick_interval_{};

 public:
  BehaviorControlNode() : Node("behavior_control_node") {
    declareParameters();
    loadParameters();

    txtLog().info(THISMODULE "Behavior Control Node starting...");
  }

  ~BehaviorControlNode() override {
    shutdown();
  }

  bool initialize() {
    if (initialized_.load()) {
      txtLog().warnning(THISMODULE "BehaviorControlNode already initialized");
      return true;
    }

    try {
      if (!initializeComponents()) {
        txtLog().error(THISMODULE "Failed to initialize components");
        return false;
      }

      if (!setupTimers()) {
        txtLog().error(THISMODULE "Failed to setup timers");
        return false;
      }

      // 设置系统状态为运行中
      mission_context_->setSystemState(behavior_core::SystemState::RUNNING);

      initialized_.store(true);
      running_.store(true);

      txtLog().info(THISMODULE "Behavior Control Node initialized successfully ");
      return true;

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Exception during initialization: %s", e.what());
      return false;
    }
  }

  void shutdown() {
    if (!initialized_.load()) return;

    txtLog().info(THISMODULE "Shutting down Behavior Control Node");

    running_.store(false);

    // 停止所有定时器
    if (status_timer_) status_timer_->cancel();
    if (watchdog_timer_) watchdog_timer_->cancel();
    if (message_process_timer_) message_process_timer_->cancel();
    if (tree_tick_timer_) tree_tick_timer_->cancel();

    // 关闭各组件
    if (behavior_executor_) {
      behavior_executor_->shutdown();
    }

    if (message_queue_) {
      message_queue_->shutdown();
    }

    if (mission_context_) {
      mission_context_->setSystemState(behavior_core::SystemState::SHUTTING_DOWN);
    }

    initialized_.store(false);
    txtLog().info(THISMODULE "Behavior Control Node shutdown complete");
  }

  bool isInitialized() const {
    return initialized_.load();
  }

 private:
  void declareParameters() {
    declare_parameter<std::string>("tree_directory", "trees");
    declare_parameter<int>("status_interval_ms", 1000);
    declare_parameter<int>("watchdog_interval_ms", 5000);
    declare_parameter<int>("message_process_interval_ms", 10);
    declare_parameter<int>("tree_tick_interval_ms", 50);
    declare_parameter<int>("message_queue_size", 100);

    // 任务参数
    declare_parameter<double>("default_takeoff_altitude", 50.0);
    declare_parameter<double>("default_navigation_speed", 2.0);
    declare_parameter<double>("default_arrival_distance", 2.0);
    declare_parameter<int>("default_loiter_duration", 10);

    // 安全参数
    declare_parameter<double>("max_altitude", 500.0);
    declare_parameter<double>("min_altitude", 5.0);
    declare_parameter<double>("geofence_radius", 1000.0);
  }

  void loadParameters() {
    tree_directory_ = get_parameter("tree_directory").as_string();
    status_interval_ = std::chrono::milliseconds(
        get_parameter("status_interval_ms").as_int());
    watchdog_interval_ = std::chrono::milliseconds(
        get_parameter("watchdog_interval_ms").as_int());
    message_process_interval_ = std::chrono::milliseconds(
        get_parameter("message_process_interval_ms").as_int());
    tree_tick_interval_ = std::chrono::milliseconds(
        get_parameter("tree_tick_interval_ms").as_int());
  }

  bool initializeComponents() {
    try {
      // 1. 数据缓存
      data_cache_ = std::make_shared<Cache>();

      // 2. 任务上下文
      mission_context_ = std::make_shared<MissionContext>();
      mission_context_->setSystemState(behavior_core::SystemState::INITIALIZING);

      // 3. 消息队列
      int queue_size = get_parameter("message_queue_size").as_int();
      message_queue_ = std::make_shared<behavior_core::MessageQueue>(queue_size);

      // 4. 行为执行器
      behavior_executor_ = std::make_shared<BehaviorExecutor>(
          shared_from_this(), tree_directory_);
      behavior_executor_->setTickInterval(tree_tick_interval_);

      // 5. ROS通信管理器
      ros_comm_ = std::make_shared<ROSCommunicationManager>(
          shared_from_this(), data_cache_, mission_context_, message_queue_);
      ros_comm_->initialize();

      // 6. 设置依赖关系
      behavior_executor_->setDependencies(ros_comm_, data_cache_, mission_context_);
      if (!behavior_executor_->initialize()) {
        txtLog().error(THISMODULE "Failed to initialize BehaviorExecutor");
        return false;
      }

      return true;

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to initialize components: %s", e.what());
      return false;
    }
  }

  bool setupTimers() {
    try {
      // 状态发布定时器
//      status_timer_ = create_wall_timer(
//          status_interval_,
//          [this]() { publishStatus(); });
//
//      // 看门狗定时器
//      watchdog_timer_ = create_wall_timer(
//          watchdog_interval_,
//          [this]() { watchdogCallback(); });

      // 消息处理定时器
      message_process_timer_ = create_wall_timer(
          message_process_interval_,
          [this]() { processMessages(); });

      // 行为树执行定时器 - 这是系统级的调度
      tree_tick_timer_ = create_wall_timer(
          tree_tick_interval_,
          [this]() { tickBehaviorTree(); });

      return true;

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to setup timers: %s", e.what());
      return false;
    }
  }

  // ================================ 定时器回调函数 ================================

  void publishStatus() {
    if (!running_.load()) return;

    try {
      // 发布系统状态
      auto status_msg = mission_context_->generateStatusMessage();
      ros_comm_->publishSystemStatus(status_msg);

      // 定期打印统计信息
      static int status_count = 0;
      if (++status_count % 10 == 0) { // 每10秒打印一次
        message_queue_->printStatistics();
        printSystemHealth();

        // 打印诊断信息
        if (status_count % 60 == 0) { // 每60秒打印详细诊断
          auto exec_stats = behavior_executor_->getExecutionStatistics();
          printExecutorStatistics(exec_stats);
        }
      }

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to publish status: %s", e.what());
    }
  }

  void watchdogCallback() {
    if (!running_.load()) return;

    try {
      // 检查各组件健康状态
      bool all_healthy = true;

      if (!behavior_executor_->isHealthy()) {
        txtLog().warnning(THISMODULE "BehaviorExecutor is not healthy");
        all_healthy = false;
      }

      if (!message_queue_->isHealthy()) {
        txtLog().warnning(THISMODULE "MessageQueue is not healthy");
        all_healthy = false;
      }

      // 检查消息队列统计
      auto stats = message_queue_->getStatistics();
      if (stats.drop_rate > 0.1) { // 丢包率超过10%
        txtLog().warnning(THISMODULE "High message drop rate: %.2f%%", stats.drop_rate * 100.0);
        all_healthy = false;
      }

      // 检查行为执行器状态
      auto exec_stats = behavior_executor_->getExecutionStatistics();
      if (exec_stats.state == behavior_core::TreeState::FAILED) {
        txtLog().warnning(THISMODULE "Behavior tree execution failed");
        all_healthy = false;
      }

      if (!all_healthy) {
        txtLog().error(THISMODULE "System health check failed");
        mission_context_->setSystemState(behavior_core::SystemState::ERROR);
        handleSystemError();
      } else {
        // 确保系统状态为运行中
        if (mission_context_->getSystemState() == behavior_core::SystemState::ERROR) {
          mission_context_->setSystemState(behavior_core::SystemState::RUNNING);
          txtLog().info(THISMODULE "System recovered from error state");
        }
      }

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Watchdog callback failed: %s", e.what());
    }
  }

  void processMessages() {
    if (!running_.load()) return;

    try {
      // 处理消息队列中的消息
      const int max_messages_per_cycle = 5; // 限制每次处理的消息数量
      int processed_count = 0;

      while (processed_count < max_messages_per_cycle) {
        auto message = message_queue_->tryPop();
        if (!message) break;

        processMessage(message);
        processed_count++;
      }

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to process messages: %s", e.what());
    }
  }

  void tickBehaviorTree() {
    if (!running_.load() || !behavior_executor_) return;

    try {
      // 执行行为树的一次tick
      behavior_executor_->tick();

      // 检查执行结果并处理状态变化
      handleTreeStateChanges();

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to tick behavior tree: %s", e.what());
    }
  }

  // ================================ 消息处理 ================================

  void processMessage(const std::shared_ptr<behavior_core::BaseMessage> &message) {
    try {
      switch (message->type) {
        case behavior_core::MessageType::TREE_EXECUTION: {
          auto tree_msg = std::dynamic_pointer_cast<behavior_core::TreeMessage>(message);
          if (tree_msg) {
            handleTreeExecution(tree_msg);
          }
          break;
        }

        case behavior_core::MessageType::COMMAND: {
          auto cmd_msg = std::dynamic_pointer_cast<behavior_core::ControlMessage>(message);
          if (cmd_msg) {
            handleControlCommand(cmd_msg);
          }
          break;
        }

        case behavior_core::MessageType::ATTACK_DESIGNATE: {
          auto attack_msg = std::dynamic_pointer_cast<behavior_core::AttackMessage>(message);
          if (attack_msg) {
            handleAttackDesignate(attack_msg);
          }
          break;
        }

        case behavior_core::MessageType::JOYSTICK_CONTROL: {
          handleJoystickControl();
          break;
        }

        default:
          //txtLog().warnning(THISMODULE "Unknown message type: %d", static_cast<int>(message->type));
          break;
      }

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to process message type %d: %s",
                     static_cast<int>(message->type), e.what());
    }
  }

  void handleTreeExecution(const std::shared_ptr<behavior_core::TreeMessage> &message) {
    txtLog().info(THISMODULE "Handling tree execution: %s for vehicle %d",
                  message->tree_name.c_str(), message->vehicle_id);

    // 检查是否是本车辆的任务
    if (message->vehicle_id != 0 && message->vehicle_id != data_cache_->getVehicleId()) {
      txtLog().debug(THISMODULE "Ignoring tree execution for vehicle %d (this is vehicle %d)",
                     message->vehicle_id, data_cache_->getVehicleId());
      return;
    }

    // 设置任务参数
    if (!message->parameters.empty()) {
      mission_context_->setParameters(message->parameters);
    }

    // 解析命令类型
    std::string cmd = "start";
    size_t dash_pos = message->tree_name.find_last_of('-');
    if (dash_pos != std::string::npos) {
      cmd = message->tree_name.substr(dash_pos + 1);
    }

    // 根据命令执行相应操作
    bool success = false;
    if (cmd == "start") {
      success = behavior_executor_->loadAndStartTree(message->tree_name, cmd);
      if (success) {
        mission_context_->incrementTasksExecuted();
      }
    } else if (cmd == "pause") {
      success = behavior_executor_->pauseTree();
    } else if (cmd == "continue") {
      success = behavior_executor_->continueTree();
    } else if (cmd == "stop") {
      success = behavior_executor_->stopTree();
    } else {
      txtLog().warnning(THISMODULE "Unknown tree command: %s", cmd.c_str());
      return;
    }

    if (success) {
      txtLog().info(THISMODULE "Successfully executed tree command: %s", cmd.c_str());
    } else {
      txtLog().error(THISMODULE "Failed to execute tree command: %s", cmd.c_str());
      mission_context_->incrementFailedTasks();
    }
  }

  void handleControlCommand(const std::shared_ptr<behavior_core::ControlMessage> &message) {
    txtLog().info(THISMODULE "Handling control command: %d for vehicle %d",
                  static_cast<int>(message->command_type), message->vehicle_id);

    // 检查是否是本车辆的命令
    if (message->vehicle_id != 0 && message->vehicle_id != data_cache_->getVehicleId()) {
      return;
    }

    switch (message->command_type) {
      case behavior_core::CommandType::STOP_TREE:behavior_executor_->stopTree();
        break;

      case behavior_core::CommandType::TAKEOFF:
        // 创建起飞任务
        createAndExecuteTask("TakeOff", "start", {{"alt", 50.0}});
        break;

      case behavior_core::CommandType::LAND:
        // 创建降落任务
        createAndExecuteTask("Land", "start", {});
        break;

      case behavior_core::CommandType::RTL:
        // 创建返航任务
        createAndExecuteTask("RtlDirect", "start", {});
        break;

      case behavior_core::CommandType::LOITER:
        // 创建悬停任务
        createAndExecuteTask("Loiter", "start", {{"duration", 30}});
        break;

      default:
        txtLog().warnning(THISMODULE "Unhandled control command: %d",
                          static_cast<int>(message->command_type));
        break;
    }
  }

  void handleAttackDesignate(const std::shared_ptr<behavior_core::AttackMessage> &message) {
    txtLog().info(THISMODULE "Handling attack designate: target_id=%d, attack_type=%d",
                  message->target_id, message->attack_type);

    // 创建攻击任务
    std::unordered_map<std::string, nlohmann::json> params = {
        {"tgtId", message->target_id},
        {"srcId", data_cache_->getVehicleId()},
        {"dstId", message->target_id}
    };

    createAndExecuteTask("Attack", "start", params);
  }

  void handleJoystickControl() {
    txtLog().debug(THISMODULE "Handling joystick control");

    // 检查是否需要触发手动控制模式
    if (data_cache_ && data_cache_->isJoyControlValid(2)) {
      // 可以根据摇杆输入触发相应的行为
      // 这里可以添加具体的摇杆控制逻辑
    }
  }

  // ================================ 行为树状态管理 ================================

  void handleTreeStateChanges() {
    auto exec_stats = behavior_executor_->getExecutionStatistics();

    static behavior_core::TreeState last_state = behavior_core::TreeState::IDLE;
    if (exec_stats.state != last_state) {
      switch (exec_stats.state) {
        case behavior_core::TreeState::COMPLETED:
          txtLog().info(THISMODULE "Tree execution completed: %s",
                        exec_stats.current_tree_name.c_str());
          mission_context_->incrementSuccessfulTasks();
          onTreeCompleted(true);
          break;

        case behavior_core::TreeState::FAILED:
          txtLog().warnning(THISMODULE "Tree execution failed: %s",
                            exec_stats.current_tree_name.c_str());
          mission_context_->incrementFailedTasks();
          onTreeCompleted(false);
          break;

        case behavior_core::TreeState::RUNNING:
          txtLog().info(THISMODULE "Tree started running: %s",
                        exec_stats.current_tree_name.c_str());
          break;

        case behavior_core::TreeState::PAUSED:
          txtLog().info(THISMODULE "Tree paused: %s",
                        exec_stats.current_tree_name.c_str());
          break;

        default:break;
      }

      last_state = exec_stats.state;
    }
  }

  static void onTreeCompleted(bool success) {
    txtLog().info(THISMODULE "Tree execution completed, success: %s", success ? "true" : "false");

    // 可以在这里添加任务完成后的处理逻辑
    // 例如：自动进行下一个任务、发送完成通知等
  }

  // ================================ 辅助方法 ================================

  void createAndExecuteTask(const std::string &task_name, const std::string &cmd,
                            const std::unordered_map<std::string, nlohmann::json> &params) {
    auto tree_msg = std::make_shared<behavior_core::TreeMessage>();
    tree_msg->tree_name = task_name + "-" + cmd;
    tree_msg->action_name = task_name;
    tree_msg->vehicle_id = data_cache_->getVehicleId();
    tree_msg->parameters = params;

    message_queue_->push(tree_msg);
  }

  void handleSystemError() {
    txtLog().warnning(THISMODULE "Handling system error - implementing recovery strategy");

    // 停止当前行为树
    behavior_executor_->stopTree();

    // 清空消息队列中的非关键消息
    message_queue_->clearMessagesOfType(behavior_core::MessageType::JOYSTICK_CONTROL);

    // 可以添加更多恢复策略，例如：
    // - 重启行为执行器
    // - 发送错误报告
    // - 尝试进入安全模式
  }

  void printSystemHealth() {
    auto exec_stats = behavior_executor_->getExecutionStatistics();

    txtLog().info(THISMODULE "System Health - State: %s, Vehicle: %d, Tree: %s, Queue: %zu/%zu",
                  mission_context_->getSystemStateString().c_str(),
                  data_cache_->getVehicleId(),
                  exec_stats.current_tree_name.c_str(),
                  message_queue_->size(),
                  message_queue_->maxSize());
  }

  static void printExecutorStatistics(const BehaviorExecutor::ExecutionStatistics &stats) {
    txtLog().info(THISMODULE "Executor Stats - Task: %s, State: %d, Ticks: %zu, Time: %ld ms, Healthy: %s",
                  stats.current_task_name.c_str(),
                  static_cast<int>(stats.state),
                  stats.total_ticks,
                  stats.execution_time.count(),
                  stats.is_healthy ? "YES" : "NO");
  }
};