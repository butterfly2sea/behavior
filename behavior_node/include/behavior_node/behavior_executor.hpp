#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"
#include "base_nodes.hpp"
#include "behavior_node/action/flight_mode_control.hpp"

// behavior_executor.hpp
class BehaviorExecutor {
 private:
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> current_tree_;
  std::shared_ptr<BT::Blackboard> blackboard_;

  std::atomic<bool> running_{false};
  std::string current_tree_name_;

 public:
  BehaviorExecutor() {
    blackboard_ = BT::Blackboard::create();
  }

  void registerNodes(std::shared_ptr<ROSCommunicationManager> ros_comm,
                     std::shared_ptr<DataCache> data_cache,
                     std::shared_ptr<MissionContext> mission_context) {

    NodeDependencies dependency{ros_comm, data_cache, mission_context};

    factory_.registerBuilder<FlightModeControl>(
        "FlightModeControl",
        [=](const std::string &name, const BT::NodeConfiguration &config) {
          return std::make_unique<FlightModeControl>(name, config, dependency);
        })
  }

  void loadTree(const std::string &tree_name) {
    if (running_) {
      stopTree();
    }

    current_tree_ = std::make_unique<BT::Tree>(
        factory_.createTree(tree_name, blackboard_));
    current_tree_name_ = tree_name;
    running_ = true;
  }

  BT::NodeStatus tick() {
    if (!current_tree_ || !running_) {
      return BT::NodeStatus::IDLE;
    }

    return current_tree_->tickOnce();
  }

  void stopTree() {
    if (current_tree_) {
      current_tree_->haltTree();
    }
    running_ = false;
  }
};