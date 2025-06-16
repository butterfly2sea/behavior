// behavior_node/include/behavior_node/core/behavior_executor.hpp
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <log/Logger.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include "behavior_node/core/types.hpp"
#include "behavior_node/core/message_queue.hpp"
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/base_nodes.hpp"

#include "behavior_node/action/flight_mode_control.hpp"
#include "behavior_node/action/lock_control.hpp"
#include "behavior_node/action/navigation_control.hpp"
#include "behavior_node/action/off_board_control.hpp"
#include "behavior_node/action/set_destination_point.hpp"
#include "behavior_node/action/set_line_parameters.hpp"
#include "behavior_node/condition/check_arrive_destination.hpp"
#include "behavior_node/condition/check_quit_search.hpp"

// 前向声明
class ROSCommunicationManager;

// 行为树执行结果枚举
enum class TreeExecutionStatus {
  SUCCESS = 0,
  FAILURE = 1,
  TIMEOUT = 2,
  HALTED = 3,
  RUNNING = 4
};

// 行为树执行结果结构
struct TreeExecutionResult {
  std::string tree_name;
  TreeExecutionStatus status;
  double duration;
  std::string error_message;
  uint32_t total_nodes_executed;
  uint32_t successful_nodes;
  uint32_t failed_nodes;
  std::chrono::steady_clock::time_point timestamp;

  // 转换为JSON
  nlohmann::json toJson() const {
    nlohmann::json j;
    j["tree_name"] = tree_name;
    j["status"] = static_cast<int>(status);
    j["duration"] = duration;
    j["error_message"] = error_message;
    j["total_nodes_executed"] = total_nodes_executed;
    j["successful_nodes"] = successful_nodes;
    j["failed_nodes"] = failed_nodes;
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        timestamp.time_since_epoch()).count();

    // 添加状态字符串描述
    switch(status) {
      case TreeExecutionStatus::SUCCESS: j["status_string"] = "SUCCESS"; break;
      case TreeExecutionStatus::FAILURE: j["status_string"] = "FAILURE"; break;
      case TreeExecutionStatus::TIMEOUT: j["status_string"] = "TIMEOUT"; break;
      case TreeExecutionStatus::HALTED: j["status_string"] = "HALTED"; break;
      case TreeExecutionStatus::RUNNING: j["status_string"] = "RUNNING"; break;
    }

    return j;
  }
};

class BehaviorExecutor {
 private:
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> current_tree_;
  std::shared_ptr<BT::Blackboard> blackboard_;

  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr message_process_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tree_result_publisher_;

  std::shared_ptr<behavior_core::MessageQueue> message_queue_;

  // 状态管理
  std::atomic<bool> is_running_{false};
  std::atomic<bool> is_paused_{false};
  std::atomic<bool> is_initialized_{false};
  std::atomic<size_t> tick_count_{0};

  std::string current_tree_name_;
  std::string tree_directory_;
  std::string tree_result_topic_;
  bool publish_tree_results_;

  std::chrono::steady_clock::time_point last_tick_time_;
  std::chrono::steady_clock::time_point tree_start_time_;
  std::chrono::milliseconds tick_interval_{50}; // 20Hz
  std::chrono::milliseconds message_check_interval_{10}; // 100Hz

  // 执行统计
  uint32_t total_nodes_executed_{0};
  uint32_t successful_nodes_{0};
  uint32_t failed_nodes_{0};

  // 依赖项
  std::shared_ptr<ROSCommunicationManager> ros_comm_;
  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;

 public:
  BehaviorExecutor(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<behavior_core::MessageQueue> msg_queue,
      std::string tree_dir = "trees")
      : node_(std::move(node)),
        message_queue_(std::move(msg_queue)),
        tree_directory_(std::move(tree_dir)) {

    blackboard_ = BT::Blackboard::create();
    last_tick_time_ = std::chrono::steady_clock::now();

    // 从参数获取配置
    publish_tree_results_ = node_->get_parameter("publish_tree_results").as_bool();
    tree_result_topic_ = node_->get_parameter("tree_result_topic").as_string();

    txtLog().info(THISMODULE "Created behavior executor");
  }

  ~BehaviorExecutor() {
    shutdown();
  }

  void setDependencies(std::shared_ptr<ROSCommunicationManager> ros_comm,
                       std::shared_ptr<Cache> data_cache,
                       std::shared_ptr<MissionContext> mission_context) {
    ros_comm_ = std::move(ros_comm);
    data_cache_ = std::move(data_cache);
    mission_context_ = std::move(mission_context);
  }

  void registerNodes() {
    if (!ros_comm_ || !data_cache_ || !mission_context_) {
      txtLog().error(THISMODULE "Dependencies not set before registering nodes");
      return;
    }

    NodeDependencies deps{ros_comm_, data_cache_, mission_context_};

    // 在这里注册具体的行为树节点
    // 飞行模式控制节点
    factory_.registerBuilder<FlightModeControl>(
        "FlightModeControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<FlightModeControl>(name, config, deps);
        });

    // 锁定控制节点
    factory_.registerBuilder<LockControl>(
        "LockControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<LockControl>(name, config, deps);
        });

    // 导航控制节点
    factory_.registerBuilder<NavigationControl>(
        "NavigationControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<NavigationControl>(name, config, deps);
        });

    // offboard控制节点
    factory_.registerBuilder<OffBoardControl>(
        "OffBoardControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<OffBoardControl>(name, config, deps);
        });

    // 设置目标点节点
    factory_.registerBuilder<SetDestinationPoint>(
        "SetDestinationPoint",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<SetDestinationPoint>(name, config, deps);
        });

    // 设置航线参数节点
    factory_.registerBuilder<SetLineParameters>(
        "SetLineParameters",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<SetLineParameters>(name, config, deps);
        });

    // 检查是否抵达目标点节点
    factory_.registerBuilder<CheckArriveDestination>(
        "CheckArriveDestination",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<CheckArriveDestination>(name, config, deps);
        });

    // 检查是否退出搜索节点
    factory_.registerBuilder<CheckQuitSearch>(
        "CheckQuitSearch",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<CheckQuitSearch>(name, config, deps);
        });

    txtLog().info(THISMODULE "Behavior tree nodes registered");
  }

  void initialize() {
    if (is_initialized_.load()) {
      txtLog().warnning(THISMODULE "BehaviorExecutor already initialized");
      return;
    }

    if (!node_) {
      txtLog().error(THISMODULE "Node not set");
      return;
    }

    // 创建定时器
    tick_timer_ = node_->create_wall_timer(
        tick_interval_,
        [this]() { tickCallback(); });

    message_process_timer_ = node_->create_wall_timer(
        message_check_interval_,
        [this]() { processMessagesCallback(); });

    // 创建行为树结果发布器
    if (publish_tree_results_) {
      tree_result_publisher_ = node_->create_publisher<std_msgs::msg::String>(
          tree_result_topic_, 10);
      txtLog().info(THISMODULE "Tree result publisher created on topic: %s", tree_result_topic_.c_str());
    }

    // 初始状态下暂停行为树定时器
    tick_timer_->cancel();

    is_initialized_.store(true);
    txtLog().info(THISMODULE "Behavior executor initialized");
  }

  void shutdown() {
    txtLog().info(THISMODULE "Shutting down behavior executor");

    if (tick_timer_) tick_timer_->cancel();
    if (message_process_timer_) message_process_timer_->cancel();

    if (current_tree_) {
      // 发布停止结果
      if (publish_tree_results_) {
        publishTreeResult(TreeExecutionStatus::HALTED, "Executor shutdown");
      }
      current_tree_->haltTree();
      current_tree_.reset();
    }

    is_running_.store(false);
    is_paused_.store(false);
    current_tree_name_.clear();

    if (mission_context_) {
      mission_context_->setCurrentTreeName("");
    }

    txtLog().info(THISMODULE "Behavior executor shutdown complete");
  }

  bool loadTree(const std::string &tree_name) {
    if (!is_initialized_.load()) {
      txtLog().error(THISMODULE "BehaviorExecutor not initialized");
      return false;
    }

    try {
      // 停止当前树
      if (current_tree_) {
        publishTreeResult(TreeExecutionStatus::HALTED, "New tree loading");
        current_tree_->haltTree();
        current_tree_.reset();
        if (tick_timer_) tick_timer_->cancel();
        is_running_.store(false);
      }

      // 构建树文件路径
      std::string tree_file = tree_directory_ + "/" + tree_name + ".xml";

      // 从文件加载树
      current_tree_ = std::make_unique<BT::Tree>(
          factory_.createTreeFromFile(tree_file, blackboard_));

      current_tree_name_ = tree_name;
      is_paused_.store(false);
      tick_count_.store(0);

      // 重置统计
      total_nodes_executed_ = 0;
      successful_nodes_ = 0;
      failed_nodes_ = 0;
      tree_start_time_ = std::chrono::steady_clock::now();

      // 更新任务上下文
      if (mission_context_) {
        mission_context_->setCurrentTreeName(tree_name);
      }

      // 启动定时器开始执行
      if (tick_timer_) {
        tick_timer_->reset();
      }
      is_running_.store(true);

      txtLog().info(THISMODULE "Successfully loaded and started behavior tree: %s",
                    tree_name.c_str());
      return true;

    } catch (const std::exception &e) {
      std::string error_msg = "Failed to load tree '" + tree_name + "': " + e.what();
      txtLog().error(THISMODULE "%s", error_msg.c_str());
      publishTreeResult(TreeExecutionStatus::FAILURE, error_msg);
      return false;
    }
  }

  void stopTree() {
    if (current_tree_) {
      publishTreeResult(TreeExecutionStatus::HALTED, "Tree stopped by request");
      current_tree_->haltTree();
      current_tree_.reset();
    }

    if (tick_timer_) {
      tick_timer_->cancel();
    }

    is_running_.store(false);
    is_paused_.store(false);
    current_tree_name_.clear();

    if (mission_context_) {
      mission_context_->setCurrentTreeName("");
    }

    txtLog().info(THISMODULE "Behavior tree stopped");
  }

  void pauseTree() {
    if (is_running_.load()) {
      is_paused_.store(true);
      if (tick_timer_) {
        tick_timer_->cancel();
      }
      txtLog().info(THISMODULE "Behavior tree paused");
    }
  }

  void resumeTree() {
    if (is_running_.load() && is_paused_.load()) {
      is_paused_.store(false);
      if (tick_timer_) {
        tick_timer_->reset();
      }
      txtLog().info(THISMODULE "Behavior tree resumed");
    }
  }

  // 状态查询
  bool isRunning() const { return is_running_.load(); }
  bool isPaused() const { return is_paused_.load(); }
  bool isInitialized() const { return is_initialized_.load(); }
  std::string getCurrentTreeName() const { return current_tree_name_; }
  size_t getTickCount() const { return tick_count_.load(); }

  void setTickInterval(std::chrono::milliseconds interval) {
    tick_interval_ = interval;

    if (tick_timer_ && is_initialized_.load()) {
      bool was_running = is_running_.load() && !is_paused_.load();

      tick_timer_->cancel();
      tick_timer_ = node_->create_wall_timer(
          tick_interval_,
          [this]() { tickCallback(); });

      if (!was_running) {
        tick_timer_->cancel();
      }
    }

    txtLog().info(THISMODULE "Tick interval set to %ld ms", interval.count());
  }

 private:
  void tickCallback() {
    if (!is_running_.load() || is_paused_.load() || !current_tree_) {
      return;
    }

    try {
      auto start_time = std::chrono::steady_clock::now();

      BT::NodeStatus status = current_tree_->tickOnce();
      tick_count_.fetch_add(1);
      last_tick_time_ = start_time;
      total_nodes_executed_++;

      handleTreeStatus(status);

      auto execution_time = std::chrono::steady_clock::now() - start_time;
      auto execution_ms = std::chrono::duration_cast<std::chrono::milliseconds>(execution_time);

      if (execution_ms > tick_interval_) {
        txtLog().warnning(THISMODULE "Tree tick took %ld ms, longer than interval %ld ms",
                          execution_ms.count(), tick_interval_.count());
      }

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Exception in tree tick: %s", e.what());
      publishTreeResult(TreeExecutionStatus::FAILURE, e.what());
      stopTree();
    }
  }

  void processMessagesCallback() {
    if (!message_queue_) return;

    behavior_core::BehaviorMessage msg{behavior_core::BehaviorCommand::NONE, "", {}};
    if (message_queue_->tryPop(msg)) {
      switch (msg.command) {
        case behavior_core::BehaviorCommand::LOAD_TREE: {
          loadTree(msg.tree_name);
          break;
        }

        case behavior_core::BehaviorCommand::STOP_TREE: {
          stopTree();
          break;
        }

        case behavior_core::BehaviorCommand::PAUSE_TREE: {
          pauseTree();
          break;
        }

        case behavior_core::BehaviorCommand::RESUME_TREE: {
          resumeTree();
          break;
        }

        case behavior_core::BehaviorCommand::SHUTDOWN: {
          shutdown();
          break;
        }

        default: {
          txtLog().warnning(THISMODULE "Unknown behavior command: %d", static_cast<int>(msg.command));
          break;
        }
      }
    }
  }

  void handleTreeStatus(BT::NodeStatus status) {
    if (status == BT::NodeStatus::SUCCESS) {
      txtLog().info(THISMODULE "Behavior tree completed successfully");
      successful_nodes_++;
      onTreeCompleted(true);
    } else if (status == BT::NodeStatus::FAILURE) {
      txtLog().warnning(THISMODULE "Behavior tree failed");
      failed_nodes_++;
      onTreeCompleted(false);
    }
    // RUNNING状态不做特殊处理，继续执行
  }

  void onTreeCompleted(bool success) {
    TreeExecutionStatus exec_status = success ? TreeExecutionStatus::SUCCESS : TreeExecutionStatus::FAILURE;
    std::string message = success ? "Tree completed successfully" : "Tree execution failed";

    publishTreeResult(exec_status, message);
    stopTree();

    txtLog().info(THISMODULE "Tree execution completed, status: %s",
                  success ? "SUCCESS" : "FAILURE");
  }

  void publishTreeResult(TreeExecutionStatus status, const std::string& error_message = "") {
    if (!publish_tree_results_ || !tree_result_publisher_) {
      return;
    }

    try {
      TreeExecutionResult result;
      result.tree_name = current_tree_name_;
      result.status = status;
      result.error_message = error_message;
      result.total_nodes_executed = total_nodes_executed_;
      result.successful_nodes = successful_nodes_;
      result.failed_nodes = failed_nodes_;
      result.timestamp = std::chrono::steady_clock::now();

      // 计算执行时长
      if (tree_start_time_.time_since_epoch().count() > 0) {
        auto duration = std::chrono::steady_clock::now() - tree_start_time_;
        result.duration = std::chrono::duration<double>(duration).count();
      } else {
        result.duration = 0.0;
      }

      // 转换为JSON并发布
      std_msgs::msg::String msg;
      msg.data = result.toJson().dump();
      tree_result_publisher_->publish(msg);

      txtLog().info(THISMODULE "Published tree result: %s - %s (%.2fs)",
                    result.tree_name.c_str(),
                    result.toJson()["status_string"].get<std::string>().c_str(),
                    result.duration);

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Failed to publish tree result: %s", e.what());
    }
  }
};