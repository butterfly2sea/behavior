#include "behavior_node/hebavior_node.hpp"

#include <rclcpp/rclcpp.hpp>

BehaviorNode::BehaviorNode(const std::string &node_name, std::string xml_file_dir)
    : rclcpp::Node(node_name){
  // 创建定时器，周期性执行行为树
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this] { data_manager_.executeCycle(); });

  RCLCPP_INFO(this->get_logger(), "Behavior节点已启动");
}

BehaviorNode::~BehaviorNode() {
  // TODO:: 停止行为树
  //data_manager_.tree_running_ = false;
  RCLCPP_INFO(this->get_logger(), "Behavior节点已关闭");
}


int main(int argc, char **argv) {
  std::string exe_path = argv[0];
  exe_path.resize(exe_path.find_last_of("/") + 1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorNode>("behavior_node", exe_path);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}