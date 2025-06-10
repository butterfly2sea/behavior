#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include "behavior_node/behavior_control_node.hpp"

std::shared_ptr<BehaviorControlNode> g_node = nullptr;

void signalHandler(int signum) {
  if (g_node) {
    txtLog().info(THISMODULE "Received signal %d, shutting down...", signum);
    g_node->shutdown();
    rclcpp::shutdown();
  }
  exit(signum);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  try {
    // 1. 先创建节点对象
    g_node = std::make_shared<BehaviorControlNode>();

    // 2. 等待节点完全被shared_ptr管理后，再进行初始化
    if (!g_node->initialize()) {
      txtLog().error(THISMODULE "Failed to initialize behavior control node");
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