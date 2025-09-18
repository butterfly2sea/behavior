#pragma once

#include <memory>
#include <string>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>

#include "behavior_node/core/behavior_executor.hpp"
#include "behavior_node/core/types.hpp"
#include "behavior_node/core/message_queue.hpp"
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"

#include <log/Logger.hpp>

/**
 * 行为节点主类 - 应用程序入口和生命周期管理
 */
class BehaviorNodeMain {
 private:
  // ROS相关
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  // 核心组件
  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;
  std::shared_ptr<ROSCommunicationManager> ros_comm_;
  std::shared_ptr<behavior_core::MessageQueue> message_queue_;
  std::unique_ptr<BehaviorExecutor> behavior_executor_;

  // 系统状态
  std::atomic<bool> is_running_{false};
  std::atomic<bool> shutdown_requested_{false};
  behavior_core::SystemConfig config_;

  // 参数
  std::string node_name_;
  std::string tree_directory_;
  std::string config_file_;

 public:
  explicit BehaviorNodeMain(const std::string& node_name = "behavior_node");
  ~BehaviorNodeMain();

  // ================================ 生命周期管理 ================================

  /**
   * 初始化节点
   * @param argc 命令行参数个数
   * @param argv 命令行参数
   * @return 是否初始化成功
   */
  bool initialize(int argc, char** argv);

  /**
   * 运行节点
   * @return 退出代码
   */
  int run();

  /**
   * 停止节点
   */
  void stop();

  /**
   * 关闭节点
   */
  void shutdown();

  // ================================ 配置管理 ================================

  /**
   * 加载配置文件
   * @param config_file 配置文件路径
   * @return 是否加载成功
   */
  bool loadConfig(const std::string& config_file);

  /**
   * 解析命令行参数
   * @param argc 参数个数
   * @param argv 参数数组
   * @return 是否解析成功
   */
  bool parseCommandLine(int argc, char** argv);

  // ================================ 状态查询 ================================

  /**
   * 检查节点是否运行中
   * @return 是否运行中
   */
  bool isRunning() const { return is_running_.load(); }

  /**
   * 获取节点名称
   * @return 节点名称
   */
  const std::string& getNodeName() const { return node_name_; }

  /**
   * 获取系统配置
   * @return 系统配置
   */
  const behavior_core::SystemConfig& getConfig() const { return config_; }

  // ================================ 信号处理 ================================

  /**
   * 设置信号处理器
   */
  static void setupSignalHandlers();

  /**
   * 信号处理函数
   * @param signal 信号编号
   */
  static void signalHandler(int signal);

 private:
  // ================================ 初始化方法 ================================

  /**
   * 初始化ROS
   * @return 是否成功
   */
  bool initializeROS();

  /**
   * 初始化组件
   * @return 是否成功
   */
  bool initializeComponents();

  /**
   * 初始化参数
   * @return 是否成功
   */
  bool initializeParameters();

  /**
   * 验证配置
   * @return 是否有效
   */
  bool validateConfig();

  // ================================ 运行控制 ================================

  /**
   * 主运行循环
   */
  void mainLoop();

  /**
   * 健康检查循环
   */
  void healthCheckLoop();

  /**
   * 处理关闭请求
   */
  void handleShutdownRequest();

  // ================================ 错误处理 ================================

  /**
   * 处理初始化错误
   * @param error_msg 错误消息
   */
  void handleInitializationError(const std::string& error_msg);

  /**
   * 处理运行时错误
   * @param error_msg 错误消息
   */
  void handleRuntimeError(const std::string& error_msg);

  /**
   * 执行紧急停止
   * @param reason 停止原因
   */
  void emergencyShutdown(const std::string& reason);

  // ================================ 辅助方法 ================================

  /**
   * 打印启动信息
   */
  void printStartupInfo();

  /**
   * 打印帮助信息
   */
  static void printHelp();

  /**
   * 打印版本信息
   */
  static void printVersion();

  /**
   * 获取默认树目录
   * @return 默认目录路径
   */
  std::string getDefaultTreeDirectory();

  /**
   * 获取默认配置文件
   * @return 默认配置文件路径
   */
  std::string getDefaultConfigFile();

  // 静态实例用于信号处理
  static BehaviorNodeMain* instance_;
};
