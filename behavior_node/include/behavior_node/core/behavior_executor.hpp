#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <log/Logger.hpp>

#include "behavior_node/core/types.hpp"
#include "behavior_node/core/message_queue.hpp"
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/base_nodes.hpp"

// 行为树节点包含
#include "behavior_node/action/flight_mode_control.hpp"
#include "behavior_node/action/lock_control.hpp"
#include "behavior_node/action/navigation_control.hpp"
#include "behavior_node/action/off_board_control.hpp"
#include "behavior_node/action/set_destination_point.hpp"
#include "behavior_node/action/set_line_parameters.hpp"
#include "behavior_node/action/trace_attack_control.hpp"
#include "behavior_node/condition/check_arrive_destination.hpp"
#include "behavior_node/condition/check_quit_search.hpp"
#include "behavior_node/condition/check_quit_line_loop.hpp"

// 任务节点包含
#include "behavior_node/action/take_off_action.hpp"
#include "behavior_node/action/land_action.hpp"
#include "behavior_node/action/loiter_action.hpp"
#include "behavior_node/action/rtl_direct_action.hpp"
#include "behavior_node/action/navline_action.hpp"
#include "behavior_node/action/attack_action.hpp"

// 前向声明
class ROSCommunicationManager;
class BehaviorExecutor {
 private:
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> current_tree_;
  std::shared_ptr<BT::Blackboard> blackboard_;

  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr message_process_timer_;

  std::shared_ptr<behavior_core::MessageQueue> message_queue_;

  // 状态管理
  std::atomic<bool> is_running_{false};
  std::atomic<bool> is_paused_{false};
  std::atomic<bool> is_initialized_{false};
  std::atomic<size_t> tick_count_{0};

  std::string current_tree_name_;
  std::string tree_directory_;

  std::chrono::steady_clock::time_point last_tick_time_;
  std::chrono::milliseconds tick_interval_{50}; // 20Hz
  std::chrono::milliseconds message_check_interval_{10}; // 100Hz

  // 依赖项
  std::shared_ptr<ROSCommunicationManager> ros_comm_;
  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;

  // 状态监控
  std::map<std::string, BT::NodeStatus> last_node_status_;
  BT::NodeStatus last_tree_status_{BT::NodeStatus::IDLE};

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

    txtLog().info(THISMODULE "Created behavior executor");
  }

  ~BehaviorExecutor() {
    shutdown();
  }

  // ================================ 初始化和配置 ================================
  void setDependencies(std::shared_ptr<ROSCommunicationManager> ros_comm,
                       std::shared_ptr<Cache> data_cache,
                       std::shared_ptr<MissionContext> mission_context) {
    ros_comm_ = std::move(ros_comm);
    data_cache_ = std::move(data_cache);
    mission_context_ = std::move(mission_context);
  }

  bool initialize() {
    if (is_initialized_.load()) {
      txtLog().warnning(THISMODULE "BehaviorExecutor already initialized");
      return true;
    }

    try {
      if (!ros_comm_ || !data_cache_ || !mission_context_) {
        txtLog().error(THISMODULE "Dependencies not set before initialization");
        return false;
      }

      registerNodes();
      // 创建定时器
      tick_timer_ = node_->create_wall_timer(
          tick_interval_,
          [this]() { tickCallback(); });

      message_process_timer_ = node_->create_wall_timer(
          message_check_interval_,
          [this]() { processMessagesCallback(); });

      // 初始状态下暂停行为树定时器
      tick_timer_->cancel();

      is_initialized_.store(true);
      txtLog().info(THISMODULE "Behavior executor initialized");
      return true;

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to initialize: %s", e.what());
      return false;
    }
  }

  void shutdown() {
    if (!is_initialized_.load()) return;

    txtLog().info(THISMODULE "Shutting down behavior executor");

    if (tick_timer_) tick_timer_->cancel();
    if (message_process_timer_) message_process_timer_->cancel();

    if (current_tree_) {
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

  // ================================ 行为树管理 ================================

  bool loadTree(const std::string &tree_name) {
    if (!is_initialized_.load()) {
      txtLog().error(THISMODULE "BehaviorExecutor not initialized");
      return false;
    }

    try {
      // 停止当前树
      if (current_tree_) {
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
      last_node_status_.clear();

      // 更新任务上下文
      if (mission_context_) {
        mission_context_->setCurrentTreeName(tree_name);
      }

      // 启动定时器开始执行
      if (tick_timer_) {
        tick_timer_->reset();
      }
      is_running_.store(true);

      txtLog().info(THISMODULE "Successfully loaded and started behavior tree: %s", tree_name.c_str());
      return true;

    } catch (const std::exception &e) {
      std::string error_msg = "Failed to load tree '" + tree_name + "': " + e.what();
      txtLog().error(THISMODULE "%s", error_msg.c_str());
      return false;
    }
  }

// ================================ 行为树执行 ================================
  void stopTree() {
    if (current_tree_) {
      current_tree_->haltTree();
      current_tree_.reset();
    }

    if (tick_timer_) {
      tick_timer_->cancel();
    }

    is_running_.store(false);
    is_paused_.store(false);
    current_tree_name_.clear();
    last_node_status_.clear();

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

      last_tree_status_ = current_tree_->tickOnce();
      tick_count_.fetch_add(1);
      last_tick_time_ = start_time;

      auto execution_time = std::chrono::steady_clock::now() - start_time;
      auto execution_ms = std::chrono::duration_cast<std::chrono::milliseconds>(execution_time);

      if (execution_ms > tick_interval_) {
        txtLog().warnning(THISMODULE "Tree tick took %ld ms, longer than interval %ld ms",
                          execution_ms.count(), tick_interval_.count());
      }

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Exception in tree tick: %s", e.what());
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
      onTreeCompleted(true);
    } else if (status == BT::NodeStatus::FAILURE) {
      txtLog().warnning(THISMODULE "Behavior tree failed");
      onTreeCompleted(false);
    }
  }

  void onTreeCompleted(bool success) {
    stopTree();
    txtLog().info(THISMODULE "Tree execution completed, status: %s",
                  success ? "SUCCESS" : "FAILURE");
  }

 private:
// ================================ 内部方法 ================================

  void registerNodes() {
    if (!ros_comm_ || !data_cache_ || !mission_context_) {
      txtLog().error(THISMODULE "Dependencies not set before registering nodes");
      return;
    }

    NodeDependencies deps{ros_comm_, data_cache_, mission_context_};

    // 注册控制节点
    registerControlNodes(deps);

    // 注册任务节点
    registerTaskNodes(deps);

    txtLog().info(THISMODULE "All behavior tree nodes registered");
  }

  void registerControlNodes(const NodeDependencies &deps) {
    // 飞行控制节点
    factory_.registerBuilder<FlightModeControl>(
        "FlightModeControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<FlightModeControl>(name, config, deps);
        });

    factory_.registerBuilder<LockControl>(
        "LockControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<LockControl>(name, config, deps);
        });

    factory_.registerBuilder<OffBoardControl>(
        "OffBoardControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<OffBoardControl>(name, config, deps);
        });

    factory_.registerBuilder<NavigationControl>(
        "NavigationControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<NavigationControl>(name, config, deps);
        });

    // 设置节点
    factory_.registerBuilder<SetDestinationPoint>(
        "SetDestinationPoint",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<SetDestinationPoint>(name, config, deps);
        });

    factory_.registerBuilder<SetLineParameters>(
        "SetLineParameters",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<SetLineParameters>(name, config, deps);
        });

    // 条件节点
    factory_.registerBuilder<CheckArriveDestination>(
        "CheckArriveDestination",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<CheckArriveDestination>(name, config, deps);
        });

    factory_.registerBuilder<CheckQuitSearch>(
        "CheckQuitSearch",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<CheckQuitSearch>(name, config, deps);
        });

    factory_.registerBuilder<CheckQuitLineLoop>(
        "CheckQuitLineLoop",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<CheckQuitLineLoop>(name, config, deps);
        });
  }

  void registerTaskNodes(const NodeDependencies &deps) {
    // 基础任务节点
    factory_.registerBuilder<TakeOffAction>(
        "TakeOffAction",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<TakeOffAction>(name, config, deps);
        });

    factory_.registerBuilder<LandAction>(
        "LandAction",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<LandAction>(name, config, deps);
        });

    factory_.registerBuilder<LoiterAction>(
        "LoiterAction",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<LoiterAction>(name, config, deps);
        });

    factory_.registerBuilder<RtlDirectAction>(
        "RtlDirectAction",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<RtlDirectAction>(name, config, deps);
        });

    factory_.registerBuilder<NavlineAction>(
        "NavlineAction",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<NavlineAction>(name, config, deps);
        });

    factory_.registerBuilder<AttackAction>(
        "AttackAction",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<AttackAction>(name, config, deps);
        });

    factory_.registerBuilder<TraceAttackControl>(
        "TraceAttackControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<TraceAttackControl>(name, config, deps);
        });
  }
};