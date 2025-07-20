#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include "behavior_node/behavior_control_node.hpp"

std::shared_ptr<BehaviorControlNode> g_node = nullptr;

void signalHandler(int signal) {
  if (g_node) {
    txtLog().info(THISMODULE "Received signal %d, shutting down gracefully", signal);
    g_node->shutdown();
    rclcpp::shutdown();
  }
  exit(signal);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 设置信号处理器
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  try {
    // 创建节点
    g_node = std::make_shared<BehaviorControlNode>();

    // 初始化
    if (!g_node->initialize()) {
      txtLog().error(THISMODULE "Failed to initialize BehaviorControlNode");
      return -1;
    }

    txtLog().info(THISMODULE "Behavior Control Node started successfully");

    // 使用单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(g_node);
    executor.spin();

  } catch (const std::exception& e) {
    txtLog().error(THISMODULE "Exception in main: %s", e.what());
    return -1;
  }

  if (g_node) {
    g_node->shutdown();
    g_node.reset();
  }

  rclcpp::shutdown();
  txtLog().info(THISMODULE "Behavior Control Node terminated");

  return 0;
}