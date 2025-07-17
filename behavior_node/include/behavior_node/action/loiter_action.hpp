#pragma once

#include "behavior_node/base_nodes.hpp"

class LoiterAction : public StatefulActionBase<LoiterAction> {
 public:
  LoiterAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<LoiterAction>(name, config, deps), duration_seconds_(10) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("duration", 10, "悬停时间(秒)")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:
  bool duration_seconds_;
  std::chrono::steady_clock::time_point hover_start_;
  bool sendLoiterCommand();

};