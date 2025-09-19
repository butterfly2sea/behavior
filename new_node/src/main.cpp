#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <memory>

#include "behavior_node/core/behavior_node_main.hpp"
#include <log/Logger.hpp>

static std::shared_ptr<BehaviorNodeMain> g_behavior_node = nullptr;

void signalHandler(int signum) {
  txtLog().warning("MAIN Signal %d received, shutting down behavior node...", signum);

  if (g_behavior_node) {
    g_behavior_node->shutdown();
    g_behavior_node.reset();
  }

  rclcpp::shutdown();
  exit(signum);
}

int main(int argc, char** argv) {
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 注册信号处理器
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  try {
    txtLog().info("MAIN Starting unified behavior node...");

    // 创建并初始化行为节点
    g_behavior_node = std::make_shared<BehaviorNodeMain>();

    if (!g_behavior_node->initialize()) {
      txtLog().error("MAIN Failed to initialize behavior node");
      return -1;
    }

    // 启动行为节点
    if (!g_behavior_node->start()) {
      txtLog().error("MAIN Failed to start behavior node");
      return -1;
    }

    txtLog().info("MAIN Behavior node started successfully");

    // 进入主循环
    g_behavior_node->run();

  } catch (const std::exception& e) {
    txtLog().error("MAIN Exception in main: %s", e.what());
    return -1;
  }

  txtLog().info("MAIN Behavior node shutdown complete");
  return 0;
}