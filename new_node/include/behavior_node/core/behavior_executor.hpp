#pragma once

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "behavior_node/core/types.hpp"
#include "behavior_node/core/message_queue.hpp"
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/base/node_dependencies.hpp"

#include <log/Logger.hpp>

/**
 * 行为树执行器 - 负责行为树的加载、执行和管理
 */
class BehaviorExecutor {
 private:
  // 核心组件
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> current_tree_;
  std::shared_ptr<BT::Blackboard> blackboard_;

  // 依赖组件
  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;
  std::shared_ptr<ROSCommunicationManager> ros_comm_;
  std::shared_ptr<behavior_core::MessageQueue> message_queue_;
  NodeDependencies node_deps_;

  // 执行控制
  std::atomic<bool> is_initialized_{false};
  std::atomic<bool> is_healthy_{true};
  std::unique_ptr<std::thread> execution_thread_;
  std::unique_ptr<std::thread> message_thread_;

  // 配置
  behavior_core::SystemConfig config_;
  behavior_core::ExecutionContext execution_context_;

  // Groot监控（可选）
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;

  // 行为树目录
  std::string tree_directory_;

 public:
  explicit BehaviorExecutor(
      std::shared_ptr<Cache> cache,
      std::shared_ptr<MissionContext> context,
      std::shared_ptr<ROSCommunicationManager> ros_comm,
      std::shared_ptr<behavior_core::MessageQueue> msg_queue,
      const behavior_core::SystemConfig& config = {});

  ~BehaviorExecutor();

  // ================================ 生命周期管理 ================================

  /**
   * 初始化执行器
   * @param tree_directory 行为树XML文件目录
   * @return 是否初始化成功
   */
  bool initialize(const std::string& tree_directory);

  /**
   * 启动执行器
   * @return 是否启动成功
   */
  bool start();

  /**
   * 停止执行器
   */
  void stop();

  /**
   * 关闭执行器
   */
  void shutdown();

  /**
   * 检查执行器是否健康
   * @return 是否健康
   */
  bool isHealthy() const { return is_healthy_.load(); }

  /**
   * 检查执行器是否已初始化
   * @return 是否已初始化
   */
  bool isInitialized() const { return is_initialized_.load(); }

  // ================================ 行为树管理 ================================

  /**
   * 加载主行为树
   * @return 是否加载成功
   */
  bool loadMainTree();

  /**
   * 获取当前执行上下文
   * @return 执行上下文
   */
  const behavior_core::ExecutionContext& getExecutionContext() const {
    return execution_context_;
  }

  /**
   * 获取当前系统状态
   * @return 系统状态
   */
  behavior_core::SystemState getSystemState() const {
    return mission_context_->getSystemState();
  }

  // ================================ 控制接口 ================================

  /**
   * 暂停执行
   * @return 是否成功
   */
  bool pause();

  /**
   * 恢复执行
   * @return 是否成功
   */
  bool resume();

  /**
   * 紧急停止
   */
  void emergencyStop();

  // ================================ 监控接口 ================================

  /**
   * 获取执行统计信息
   */
  struct ExecutionStats {
    uint64_t total_ticks = 0;
    std::chrono::seconds uptime{0};
    uint32_t error_count = 0;
    double average_tick_time_ms = 0.0;
    std::string current_tree_name;
    behavior_core::TreeState current_state = behavior_core::TreeState::IDLE;
  };

  ExecutionStats getExecutionStats() const;

 private:
  // ================================ 初始化方法 ================================

  /**
   * 注册所有行为节点
   */
  void registerAllNodes();

  /**
   * 注册飞行动作节点
   */
  void registerFlightActions();

  /**
   * 注册导航动作节点
   */
  void registerNavigationActions();

  /**
   * 注册搜索动作节点
   */
  void registerSearchActions();

  /**
   * 注册控制动作节点
   */
  void registerControlActions();

  /**
   * 注册编队动作节点
   */
  void registerFormationActions();

  /**
   * 注册条件节点
   */
  void registerConditionNodes();

  /**
   * 设置黑板参数
   */
  void setupBlackboard();

  /**
   * 初始化监控
   */
  void initializeMonitoring();

  // ================================ 执行方法 ================================

  /**
   * 主执行线程
   */
  void executionLoop();

  /**
   * 消息处理线程
   */
  void messageLoop();

  /**
   * 执行单次tick
   * @return 执行结果
   */
  BT::NodeStatus executeTick();

  /**
   * 处理行为消息
   * @param message 待处理的消息
   */
  void processMessage(const behavior_core::BehaviorMessage& message);

  /**
   * 处理树加载消息
   * @param tree_name 树名称
   * @param params 参数
   */
  void handleLoadTree(const std::string& tree_name,
                      const std::unordered_map<std::string, nlohmann::json>& params);

  /**
   * 处理树停止消息
   */
  void handleStopTree();

  /**
   * 处理树暂停消息
   */
  void handlePauseTree();

  /**
   * 处理树恢复消息
   */
  void handleResumeTree();

  /**
   * 处理参数设置消息
   * @param params 参数映射
   */
  void handleSetParameter(const std::unordered_map<std::string, nlohmann::json>& params);

  // ================================ 状态管理 ================================

  /**
   * 更新执行状态
   * @param state 新状态
   */
  void updateTreeState(behavior_core::TreeState state);

  /**
   * 更新系统状态
   * @param state 新状态
   */
  void updateSystemState(behavior_core::SystemState state);

  /**
   * 检查健康状态
   * @return 是否健康
   */
  bool checkHealth();

  /**
   * 处理错误
   * @param error_msg 错误消息
   */
  void handleError(const std::string& error_msg);

  // ================================ 辅助方法 ================================

  /**
   * 获取树文件路径
   * @param tree_name 树名称
   * @return 完整文件路径
   */
  std::string getTreeFilePath(const std::string& tree_name) const;

  /**
   * 验证树名称
   * @param tree_name 树名称
   * @return 是否有效
   */
  bool isValidTreeName(const std::string& tree_name) const;

  /**
   * 清理资源
   */
  void cleanup();

  /**
   * 记录执行统计
   */
  void updateExecutionStats();
};
