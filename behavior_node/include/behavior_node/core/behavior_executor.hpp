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
#include "behavior_node/action/joy_control.hpp"
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

namespace behavior_core {

// 行为树状态枚举
enum class TreeState {
  IDLE,           // 空闲状态
  LOADING,        // 加载中
  RUNNING,        // 运行中
  PAUSED,         // 暂停
  COMPLETED,      // 完成
  FAILED,         // 失败
  STOPPING        // 停止中
};

// 行为树执行上下文
struct TreeExecutionContext {
  std::string tree_name;
  std::string task_name;
  TreeState state{TreeState::IDLE};
  std::chrono::steady_clock::time_point start_time;
  std::chrono::steady_clock::time_point last_tick_time;
  size_t tick_count{0};
  BT::NodeStatus last_status{BT::NodeStatus::IDLE};
  std::unordered_map<std::string, nlohmann::json> parameters;
};

} // namespace behavior_core

class BehaviorExecutor {
 private:
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> current_tree_;
  std::shared_ptr<BT::Blackboard> blackboard_;

  // 执行上下文
  behavior_core::TreeExecutionContext execution_context_;

  // 基础组件
  rclcpp::Node::SharedPtr node_;
  std::string tree_directory_;

  // 依赖项
  std::shared_ptr<ROSCommunicationManager> ros_comm_;
  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;

  // 状态管理
  std::atomic<bool> is_initialized_{false};
  std::chrono::milliseconds tick_interval_{50}; // 20Hz

  // 健康状态监控
  std::atomic<bool> is_healthy_{true};
  std::chrono::steady_clock::time_point last_healthy_check_;
  std::chrono::milliseconds health_check_interval_{1000};

 public:
  BehaviorExecutor(rclcpp::Node::SharedPtr node, std::string tree_dir = "trees")
      : node_(std::move(node)), tree_directory_(std::move(tree_dir)) {

    blackboard_ = BT::Blackboard::create();
    last_healthy_check_ = std::chrono::steady_clock::now();

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
      is_initialized_.store(true);
      is_healthy_.store(true);

      txtLog().info(THISMODULE "Behavior executor initialized");
      return true;

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Failed to initialize: %s", e.what());
      return false;
    }
  }

  void shutdown() {
    if (!is_initialized_.load()) return;

    txtLog().info(THISMODULE "Shutting down behavior executor");

    stopTree();
    is_initialized_.store(false);
    is_healthy_.store(false);

    txtLog().info(THISMODULE "Behavior executor shutdown complete");
  }

  // ================================ 行为树管理 ================================

  bool loadAndStartTree(const std::string& tree_name, const std::string& cmd = "start") {
    if (!is_initialized_.load()) {
      txtLog().error(THISMODULE "BehaviorExecutor not initialized");
      return false;
    }

    try {
      // 构建完整的行为树名称
      std::string full_tree_name = tree_name;
      if (full_tree_name.find('-') == std::string::npos) {
        full_tree_name += "-" + cmd;
      }

      // 停止当前树
      if (execution_context_.state != behavior_core::TreeState::IDLE) {
        stopTree();
      }

      // 更新执行上下文
      execution_context_.tree_name = full_tree_name;
      execution_context_.task_name = tree_name;
      execution_context_.state = behavior_core::TreeState::LOADING;
      execution_context_.start_time = std::chrono::steady_clock::now();
      execution_context_.tick_count = 0;

      // 构建树文件路径
      std::string tree_file = tree_directory_ + "/" + full_tree_name + ".xml";

      // 从文件加载树
      current_tree_ = std::make_unique<BT::Tree>(
          factory_.createTreeFromFile(tree_file, blackboard_));

      // 更新任务上下文
      if (mission_context_) {
        mission_context_->setCurrentTreeName(full_tree_name);
      }

      // 根据命令设置状态
      if (cmd == "start") {
        execution_context_.state = behavior_core::TreeState::RUNNING;
      } else if (cmd == "pause") {
        execution_context_.state = behavior_core::TreeState::PAUSED;
      } else {
        execution_context_.state = behavior_core::TreeState::RUNNING;
      }

      txtLog().info(THISMODULE "Successfully loaded tree: %s (cmd: %s)",
                    full_tree_name.c_str(), cmd.c_str());
      return true;

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Failed to load tree '%s': %s",
                     tree_name.c_str(), e.what());
      execution_context_.state = behavior_core::TreeState::FAILED;
      return false;
    }
  }

  // ================================ 行为树状态控制 ================================

  bool startTree() {
    if (execution_context_.state == behavior_core::TreeState::PAUSED ||
        execution_context_.state == behavior_core::TreeState::IDLE) {
      execution_context_.state = behavior_core::TreeState::RUNNING;
      txtLog().info(THISMODULE "Tree started/resumed: %s",
                    execution_context_.tree_name.c_str());
      return true;
    }
    return false;
  }

  bool pauseTree() {
    if (execution_context_.state == behavior_core::TreeState::RUNNING) {
      execution_context_.state = behavior_core::TreeState::PAUSED;
      if (current_tree_) {
        current_tree_->haltTree();
      }
      txtLog().info(THISMODULE "Tree paused: %s",
                    execution_context_.tree_name.c_str());
      return true;
    }
    return false;
  }

  bool continueTree() {
    if (execution_context_.state == behavior_core::TreeState::PAUSED) {
      execution_context_.state = behavior_core::TreeState::RUNNING;
      txtLog().info(THISMODULE "Tree continued: %s",
                    execution_context_.tree_name.c_str());
      return true;
    }
    return false;
  }

  bool stopTree() {
    if (execution_context_.state != behavior_core::TreeState::IDLE) {
      execution_context_.state = behavior_core::TreeState::STOPPING;

      if (current_tree_) {
        current_tree_->haltTree();
        current_tree_.reset();
      }

      execution_context_.state = behavior_core::TreeState::IDLE;
      execution_context_.tree_name.clear();
      execution_context_.task_name.clear();
      execution_context_.tick_count = 0;

      txtLog().info(THISMODULE "Tree stopped");
      return true;
    }
    return false;
  }

  // ================================ 行为树执行 ================================

  bool tick() {
    if (!current_tree_ || execution_context_.state != behavior_core::TreeState::RUNNING) {
      return false;
    }

    try {
      auto start_time = std::chrono::steady_clock::now();

      BT::NodeStatus status = current_tree_->tickOnce();
      execution_context_.tick_count++;
      execution_context_.last_tick_time = start_time;
      execution_context_.last_status = status;

      // 处理行为树状态
      handleTreeTickResult(status);

      // 更新健康状态
      updateHealthStatus();

      auto execution_time = std::chrono::steady_clock::now() - start_time;
      auto execution_ms = std::chrono::duration_cast<std::chrono::milliseconds>(execution_time);

      if (execution_ms > tick_interval_) {
        txtLog().warnning(THISMODULE "Tree tick took %ld ms, longer than interval %ld ms",
                          execution_ms.count(), tick_interval_.count());
      }

      return true;

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Exception in tree tick: %s", e.what());
      execution_context_.state = behavior_core::TreeState::FAILED;
      execution_context_.last_status = BT::NodeStatus::FAILURE;
      return false;
    }
  }

  // ================================ 状态查询 ================================

  bool isInitialized() const { return is_initialized_.load(); }
  bool isHealthy() const {
    // 检查基本健康状态
    if (!is_healthy_.load()) return false;

    // 检查是否长时间没有健康检查
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_healthy_check_);
    if (elapsed > health_check_interval_ * 5) { // 5倍间隔认为不健康
      return false;
    }

    return true;
  }

  bool isRunning() const {
    return execution_context_.state == behavior_core::TreeState::RUNNING;
  }

  bool isPaused() const {
    return execution_context_.state == behavior_core::TreeState::PAUSED;
  }

  behavior_core::TreeState getTreeState() const {
    return execution_context_.state;
  }

  std::string getCurrentTreeName() const {
    return execution_context_.tree_name;
  }

  std::string getCurrentTaskName() const {
    return execution_context_.task_name;
  }

  size_t getTickCount() const {
    return execution_context_.tick_count;
  }

  BT::NodeStatus getLastStatus() const {
    return execution_context_.last_status;
  }

  // ================================ 配置管理 ================================

  void setTickInterval(std::chrono::milliseconds interval) {
    tick_interval_ = interval;
    txtLog().info(THISMODULE "Tick interval set to %ld ms", interval.count());
  }

  std::chrono::milliseconds getTickInterval() const {
    return tick_interval_;
  }

  void setTreeDirectory(const std::string& directory) {
    tree_directory_ = directory;
    txtLog().info(THISMODULE "Tree directory set to: %s", directory.c_str());
  }

  // ================================ 统计信息 ================================

  struct ExecutionStatistics {
    std::string current_tree_name;
    std::string current_task_name;
    behavior_core::TreeState state;
    size_t total_ticks{};
    std::chrono::milliseconds execution_time{};
    BT::NodeStatus last_status;
    bool is_healthy{};
  };

  ExecutionStatistics getExecutionStatistics() const {
    ExecutionStatistics stats;
    stats.current_tree_name = execution_context_.tree_name;
    stats.current_task_name = execution_context_.task_name;
    stats.state = execution_context_.state;
    stats.total_ticks = execution_context_.tick_count;

    if (execution_context_.state != behavior_core::TreeState::IDLE) {
      auto now = std::chrono::steady_clock::now();
      stats.execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - execution_context_.start_time);
    } else {
      stats.execution_time = std::chrono::milliseconds{0};
    }

    stats.last_status = execution_context_.last_status;
    stats.is_healthy = isHealthy();

    return stats;
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

  void registerControlNodes(const NodeDependencies& deps) {
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

    factory_.registerBuilder<JoyControl>(
        "JoyControl",
        [deps](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<JoyControl>(name, config, deps);
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

  void registerTaskNodes(const NodeDependencies& deps) {
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

  void handleTreeTickResult(BT::NodeStatus status) {
    switch (status) {
      case BT::NodeStatus::SUCCESS:
        txtLog().info(THISMODULE "Tree completed successfully: %s",
                      execution_context_.tree_name.c_str());
        execution_context_.state = behavior_core::TreeState::COMPLETED;
        break;

      case BT::NodeStatus::FAILURE:
        txtLog().warnning(THISMODULE "Tree failed: %s",
                          execution_context_.tree_name.c_str());
        execution_context_.state = behavior_core::TreeState::FAILED;
        break;

      case BT::NodeStatus::RUNNING:
        // 继续运行，无需特殊处理
        break;

      default:
        txtLog().warnning(THISMODULE "Unexpected tree status: %d",
                          static_cast<int>(status));
        break;
    }
  }

  void updateHealthStatus() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_healthy_check_);

    if (elapsed >= health_check_interval_) {
      // 执行健康检查
      bool healthy = true;

      // 检查依赖项是否有效
      if (!ros_comm_ || !data_cache_ || !mission_context_) {
        healthy = false;
      }

      // 检查执行频率是否正常
      if (execution_context_.state == behavior_core::TreeState::RUNNING) {
        auto tick_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - execution_context_.last_tick_time);
        if (tick_elapsed > tick_interval_ * 5) { // 5倍间隔认为异常
          healthy = false;
        }
      }

      is_healthy_.store(healthy);
      last_healthy_check_ = now;
    }
  }
};