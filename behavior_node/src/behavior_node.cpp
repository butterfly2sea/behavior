#include <rclcpp/rclcpp.hpp>
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/behavior_executor.hpp"
#include "base_nodes.hpp"

class BehaviorControlNode : public rclcpp::Node {
 private:
  // 核心组件
  std::shared_ptr<DataCache> data_cache_;
  std::shared_ptr<ROSCommunicationManager> ros_comm_;
  std::shared_ptr<MissionContext> mission_context_;
  std::shared_ptr<BehaviorExecutor> behavior_executor_;

  // 定时器
  rclcpp::TimerBase::SharedPtr tick_timer_;

 public:
  BehaviorControlNode() : Node("behavior_control") {
    // 初始化组件
    data_cache_ = std::make_shared<DataCache>();
    ros_comm_ = std::make_shared<ROSCommunicationManager>(
        shared_from_this(), data_cache_);
    mission_context_ = std::make_shared<MissionContext>();
    behavior_executor_ = std::make_shared<BehaviorExecutor>();

    // 初始化
    ros_comm_->initialize();

    // 创建依赖容器
    NodeDependencies deps{ros_comm_, data_cache_, mission_context_};

    // 注册行为树节点
//    behavior_executor_->registerNodes(deps);

    // 创建定时器
    tick_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { tick(); });
  }

 private:
  void tick() {
    auto status = behavior_executor_->tick();

    if (status == BT::NodeStatus::SUCCESS ||
        status == BT::NodeStatus::FAILURE) {
      // 行为树执行完成
      handleTreeCompletion(status);
    }
  }

  void handleTreeCompletion(BT::NodeStatus status) {
    // 处理行为树完成逻辑
    txtLog().info(THISMODULE  "Tree completed with status: %s",
                status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}