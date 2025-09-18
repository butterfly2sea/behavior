#include "behavior_node/core/behavior_node_main.hpp"

#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>

// 静态成员初始化
BehaviorNodeMain* BehaviorNodeMain::instance_ = nullptr;

BehaviorNodeMain::BehaviorNodeMain(const std::string& node_name)
    : node_name_(node_name) {
  instance_ = this;

  // 设置默认配置
  config_.tree_directory = "tree";
  config_.main_tree_file = "main_behavior_tree.xml";
  config_.tick_rate = 10.0;
  config_.max_tick_count = 1000000;
  config_.timeout_duration = std::chrono::seconds(30);
  config_.enable_groot_monitoring = false;
  config_.groot_port = 1666;

  txtLog().info(THISMODULE "BehaviorNodeMain created with name: %s", node_name_.c_str());
}

BehaviorNodeMain::~BehaviorNodeMain() {
  shutdown();
  instance_ = nullptr;
  txtLog().info(THISMODULE "BehaviorNodeMain destroyed");
}

bool BehaviorNodeMain::initialize(int argc, char** argv) {
  txtLog().info(THISMODULE "Initializing BehaviorNodeMain...");

  try {
    // 解析命令行参数
    if (!parseCommandLine(argc, argv)) {
      return false;
    }

    // 初始化ROS
    if (!initializeROS()) {
      handleInitializationError("Failed to initialize ROS");
      return false;
    }

    // 初始化参数
    if (!initializeParameters()) {
      handleInitializationError("Failed to initialize parameters");
      return false;
    }

    // 验证配置
    if (!validateConfig()) {
      handleInitializationError("Invalid configuration");
      return false;
    }

    // 初始化组件
    if (!initializeComponents()) {
      handleInitializationError("Failed to initialize components");
      return false;
    }

    txtLog().info(THISMODULE "BehaviorNodeMain initialized successfully");
    printStartupInfo();

    return true;

  } catch (const std::exception& e) {
    handleInitializationError(std::string("Exception during initialization: ") + e.what());
    return false;
  }
}

int BehaviorNodeMain::run() {
  if (!is_running_.load()) {
    txtLog().error(THISMODULE "Cannot run - node not properly initialized");
    return 1;
  }

  txtLog().info(THISMODULE "Starting BehaviorNodeMain execution...");

  try {
    // 启动行为执行器
    if (!behavior_executor_->start()) {
      txtLog().error(THISMODULE "Failed to start behavior executor");
      return 1;
    }

    // 启动健康检查线程
    std::thread health_check_thread(&BehaviorNodeMain::healthCheckLoop, this);

    // 主运行循环
    mainLoop();

    // 等待健康检查线程结束
    if (health_check_thread.joinable()) {
      health_check_thread.join();
    }

    txtLog().info(THISMODULE "BehaviorNodeMain execution completed");
    return 0;

  } catch (const std::exception& e) {
    handleRuntimeError(std::string("Runtime exception: ") + e.what());
    return 1;
  }
}

void BehaviorNodeMain::stop() {
  txtLog().info(THISMODULE "Stopping BehaviorNodeMain...");

  shutdown_requested_.store(true);

  if (behavior_executor_) {
    behavior_executor_->stop();
  }

  if (executor_) {
    executor_->cancel();
  }
}

void BehaviorNodeMain::shutdown() {
  if (!is_running_.load()) {
    return;
  }

  txtLog().info(THISMODULE "Shutting down BehaviorNodeMain...");

  is_running_.store(false);
  shutdown_requested_.store(true);

  // 停止行为执行器
  if (behavior_executor_) {
    behavior_executor_->shutdown();
    behavior_executor_.reset();
  }

  // 停止ROS通信
  if (ros_comm_) {
    ros_comm_.reset();
  }

  // 停止执行器
  if (executor_) {
    executor_->cancel();
    executor_.reset();
  }

  // 清理ROS
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  txtLog().info(THISMODULE "BehaviorNodeMain shutdown complete");
}

bool BehaviorNodeMain::parseCommandLine(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "-c" || arg == "--config") {
      if (i + 1 < argc) {
        config_file_ = argv[++i];
      }
    }
    else if (arg == "-t" || arg == "--tree-dir") {
      if (i + 1 < argc) {
        tree_directory_ = argv[++i];
      }
    }
  }

  return true;
}

bool BehaviorNodeMain::initializeROS() {
  try {
    // 初始化ROS2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // 创建ROS节点
    ros_node_ = rclcpp::Node::make_shared(node_name_);
    if (!ros_node_) {
      txtLog().error(THISMODULE "Failed to create ROS node");
      return false;
    }

    // 创建执行器
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(ros_node_);

    txtLog().info(THISMODULE "ROS initialized successfully");
    return true;

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "ROS initialization failed: %s", e.what());
    return false;
  }
}

bool BehaviorNodeMain::initializeParameters() {
  try {
    // 从ROS参数服务器获取参数
    ros_node_->declare_parameter("tree_directory", config_.tree_directory);
    ros_node_->declare_parameter("main_tree_file", config_.main_tree_file);
    ros_node_->declare_parameter("tick_rate", config_.tick_rate);
    ros_node_->declare_parameter("enable_groot_monitoring", config_.enable_groot_monitoring);
    ros_node_->declare_parameter("groot_port", static_cast<int>(config_.groot_port));

    config_.tree_directory = ros_node_->get_parameter("tree_directory").as_string();
    config_.main_tree_file = ros_node_->get_parameter("main_tree_file").as_string();
    config_.tick_rate = ros_node_->get_parameter("tick_rate").as_double();
    config_.enable_groot_monitoring = ros_node_->get_parameter("enable_groot_monitoring").as_bool();
    config_.groot_port = static_cast<uint16_t>(ros_node_->get_parameter("groot_port").as_int());

    // 如果指定了树目录，使用命令行参数
    if (!tree_directory_.empty()) {
      config_.tree_directory = tree_directory_;
    }

    txtLog().info(THISMODULE "Parameters initialized successfully");
    return true;

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Parameter initialization failed: %s", e.what());
    return false;
  }
}

bool BehaviorNodeMain::validateConfig() {
  // 检查树目录是否存在
  if (!std::filesystem::exists(config_.tree_directory)) {
    txtLog().error(THISMODULE "Tree directory does not exist: %s", config_.tree_directory.c_str());
    return false;
  }

  // 检查主树文件是否存在
  std::string main_tree_path = config_.tree_directory + "/" + config_.main_tree_file;
  if (!std::filesystem::exists(main_tree_path)) {
    txtLog().error(THISMODULE "Main tree file does not exist: %s", main_tree_path.c_str());
    return false;
  }

  // 验证tick频率
  if (config_.tick_rate <= 0.0 || config_.tick_rate > 1000.0) {
    txtLog().error(THISMODULE "Invalid tick rate: %f", config_.tick_rate);
    return false;
  }

  txtLog().info(THISMODULE "Configuration validated successfully");
  return true;
}

bool BehaviorNodeMain::initializeComponents() {
  try {
    // 创建数据缓存
    data_cache_ = std::make_shared<Cache>();

    // 创建任务上下文
    mission_context_ = std::make_shared<MissionContext>();

    // 创建消息队列
    message_queue_ = std::make_shared<behavior_core::MessageQueue>(1000);

    // 创建ROS通信管理器
    ros_comm_ = std::make_shared<ROSCommunicationManager>(
        ros_node_, data_cache_, mission_context_, message_queue_);

    // 初始化ROS通信
    ros_comm_->initialize();

    // 创建行为执行器
    behavior_executor_ = std::make_unique<BehaviorExecutor>(
        data_cache_, mission_context_, ros_comm_, message_queue_, config_);

    // 初始化行为执行器
    if (!behavior_executor_->initialize(config_.tree_directory)) {
      txtLog().error(THISMODULE "Failed to initialize behavior executor");
      return false;
    }

    is_running_.store(true);

    txtLog().info(THISMODULE "All components initialized successfully");
    return true;

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Component initialization failed: %s", e.what());
    return false;
  }
}

void BehaviorNodeMain::mainLoop() {
  txtLog().info(THISMODULE "Entering main execution loop");

  while (is_running_.load() && !shutdown_requested_.load() && rclcpp::ok()) {
    try {
      // 执行ROS spin
      executor_->spin_once(std::chrono::milliseconds(100));

      // 检查行为执行器健康状态
      if (!behavior_executor_->isHealthy()) {
        txtLog().warning(THISMODULE "Behavior executor is unhealthy");
        handleRuntimeError("Behavior executor health check failed");
      }

      // 短暂休眠避免CPU占用过高
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Exception in main loop: %s", e.what());
      handleRuntimeError(std::string("Main loop exception: ") + e.what());
      break;
    }
  }

  txtLog().info(THISMODULE "Exiting main execution loop");
}

void BehaviorNodeMain::healthCheckLoop() {
  txtLog().info(THISMODULE "Starting health check loop");

  while (is_running_.load() && !shutdown_requested_.load()) {
    try {
      // 检查系统资源
      // 检查ROS连接状态
      // 检查行为执行器状态

      std::this_thread::sleep_for(std::chrono::seconds(5));

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Exception in health check: %s", e.what());
    }
  }

  txtLog().info(THISMODULE "Health check loop finished");
}

void BehaviorNodeMain::handleShutdownRequest() {
  txtLog().info(THISMODULE "Processing shutdown request");
  shutdown_requested_.store(true);
}

void BehaviorNodeMain::handleInitializationError(const std::string& error_msg) {
  txtLog().error(THISMODULE "Initialization error: %s", error_msg.c_str());
  is_running_.store(false);
}

void BehaviorNodeMain::handleRuntimeError(const std::string& error_msg) {
  txtLog().error(THISMODULE "Runtime error: %s", error_msg.c_str());

  // 根据错误严重程度决定是否紧急关闭
  if (error_msg.find("critical") != std::string::npos ||
      error_msg.find("fatal") != std::string::npos) {
    emergencyShutdown(error_msg);
  }
}

void BehaviorNodeMain::emergencyShutdown(const std::string& reason) {
  txtLog().error(THISMODULE "EMERGENCY SHUTDOWN: %s", reason.c_str());

  // 立即停止所有操作
  if (behavior_executor_) {
    behavior_executor_->emergencyStop();
  }

  shutdown_requested_.store(true);
  is_running_.store(false);
}

void BehaviorNodeMain::printStartupInfo() {
  txtLog().info(THISMODULE "=== Behavior Node Startup Information ===");
  txtLog().info(THISMODULE "Node Name: %s", node_name_.c_str());
  txtLog().info(THISMODULE "Tree Directory: %s", config_.tree_directory.c_str());
  txtLog().info(THISMODULE "Main Tree File: %s", config_.main_tree_file.c_str());
  txtLog().info(THISMODULE "Tick Rate: %.1f Hz", config_.tick_rate);
  txtLog().info(THISMODULE "Groot Monitoring: %s", config_.enable_groot_monitoring ? "Enabled" : "Disabled");
  if (config_.enable_groot_monitoring) {
    txtLog().info(THISMODULE "Groot Port: %d", config_.groot_port);
  }
  txtLog().info(THISMODULE "========================================");
}

// 静态方法实现
void BehaviorNodeMain::setupSignalHandlers() {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGQUIT, signalHandler);
}

void BehaviorNodeMain::signalHandler(int signal) {
  if (instance_) {
    instance_->handleShutdownRequest();
  }
}

void BehaviorNodeMain::printHelp() {
  std::cout << "Behavior Node - UAV Behavior Control System\n";
  std::cout << "Usage: behavior_node [OPTIONS]\n";
  std::cout << "\nOptions:\n";
  std::cout << "  -h, --help              Show this help\n";
  std::cout << "  -v, --version           Show version info\n";
  std::cout << "  -c, --config FILE       Configuration file\n";
  std::cout << "  -t, --tree-dir DIR      Tree directory\n";
  std::cout << "\n";
}

void BehaviorNodeMain::printVersion() {
  std::cout << "Behavior Node v1.0.0\n";
  std::cout << "Built with BehaviorTree.CPP and ROS2\n";
}

std::string BehaviorNodeMain::getDefaultTreeDirectory() {
  return "tree";
}

std::string BehaviorNodeMain::getDefaultConfigFile() {
  return "config/behavior_node.yaml";
}
