#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <atomic>

#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/core/behavior_executor.hpp"
#include "behavior_node/core/message_queue.hpp"

class BehaviorControlNode : public rclcpp::Node {
 private:
  // 核心组件
  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;
  std::shared_ptr<behavior_core::MessageQueue> message_queue_;
  std::shared_ptr<BehaviorExecutor> behavior_executor_;
  std::shared_ptr<ROSCommunicationManager> ros_comm_;

  // 定时器
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // 系统状态
  std::atomic<bool> initialized_{false};
  std::atomic<bool> shutting_down_{false};

  // 配置参数
  std::string tree_directory_;
  std::chrono::milliseconds status_interval_{1000};
  std::chrono::milliseconds watchdog_interval_{5000};

 public:
  explicit BehaviorControlNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("behavior_control", options) {

    declareParameters();
    loadParameters();

    txtLog().info(THISMODULE "Behavior Control Node created, ready for initialization");
  }

  ~BehaviorControlNode() override {
    shutdown();
  }

  // 新增：独立的初始化方法
  bool initialize() {
    if (initialized_.load()) {
      txtLog().warnning(THISMODULE "Node already initialized");
      return true;
    }

    if (!initializeComponents()) {
      txtLog().error(THISMODULE "Failed to initialize components");
      return false;
    }

    setupTimers();

    initialized_.store(true);
    txtLog().info(THISMODULE "Behavior Control Node initialized successfully");
    return true;
  }

  void shutdown() {
    if (shutting_down_.exchange(true)) {
      return;
    }

    txtLog().info(THISMODULE "Shutting down Behavior Control Node");

    if (status_timer_) status_timer_->cancel();
    if (watchdog_timer_) watchdog_timer_->cancel();

    if (behavior_executor_) {
      behavior_executor_->shutdown();
    }

    if (message_queue_) {
      message_queue_->shutdown();
    }

    if (mission_context_) {
      mission_context_->setSystemState(behavior_core::SystemState::SHUTTING_DOWN);
    }

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
    declare_parameter<int>("tick_interval_ms", 50);
    declare_parameter<int>("message_queue_size", 100);
  }

  void loadParameters() {
    tree_directory_ = get_parameter("tree_directory").as_string();
    status_interval_ = std::chrono::milliseconds(
        get_parameter("status_interval_ms").as_int());
    watchdog_interval_ = std::chrono::milliseconds(
        get_parameter("watchdog_interval_ms").as_int());
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
      behavior_executor_ = std::make_shared<BehaviorExecutor>(shared_from_this(), message_queue_, tree_directory_);

      // 5. ROS通信管理器
      ros_comm_ = std::make_shared<ROSCommunicationManager>(
          shared_from_this(), data_cache_, mission_context_,
          message_queue_);

      // 设置依赖关系
      behavior_executor_->setDependencies(ros_comm_, data_cache_, mission_context_);

      // 初始化组件
      ros_comm_->initialize();
      behavior_executor_->registerNodes();
      behavior_executor_->initialize();

      // 设置行为执行器参数
      int tick_interval = get_parameter("tick_interval_ms").as_int();
      behavior_executor_->setTickInterval(std::chrono::milliseconds(tick_interval));

      // 更新系统状态
      mission_context_->setSystemState(behavior_core::SystemState::READY);

      return true;

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to initialize components: %s", e.what());
      return false;
    }
  }

  void setupTimers() {
    status_timer_ = create_wall_timer(
        status_interval_,
        [this]() { publishStatus(); });

    watchdog_timer_ = create_wall_timer(
        watchdog_interval_,
        [this]() { watchdogCheck(); });
  }

  void publishStatus() {
    if (!mission_context_ || !data_cache_ || !behavior_executor_) return;

    auto system_state = mission_context_->getSystemState();
    bool executor_running = behavior_executor_->isRunning();
    std::string current_tree = behavior_executor_->getCurrentTreeName();

//    txtLog().debug(
//        THISMODULE
//        "Status - System: %d, Vehicle: %s, Executor: %s, Tree: %s",
//        static_cast<int>(system_state),
//        data_cache_->isVehicleStateValid() ? "OK" : "INVALID",
//        executor_running ? "RUNNING" : "STOPPED",
//        current_tree.c_str());
  }

  void watchdogCheck() {
    if (!data_cache_) return;

//    if (!data_cache_->isVehicleStateValid()) {
//      txtLog().warnning(THISMODULE "Vehicle state data is stale or invalid");
//    }

    // 检查行为执行器状态
    if (behavior_executor_) {
      size_t tick_count = behavior_executor_->getTickCount();
      static size_t last_tick_count = 0;

      if (behavior_executor_->isRunning() && tick_count == last_tick_count) {
        txtLog().warnning(THISMODULE "Behavior executor may be stuck");
      }
      last_tick_count = tick_count;
    }

    // 检查消息队列状态
    if (message_queue_ && message_queue_->size() > 50) {
      txtLog().warnning(THISMODULE "Message queue is filling up: %zu messages", message_queue_->size());
    }
  }
};