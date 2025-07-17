#pragma once

#include "behavior_node/base_nodes.hpp"

// ================================ 起飞节点 ================================
class TakeOffAction : public StatefulActionBase<TakeOffAction> {
 public:
  TakeOffAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<TakeOffAction>(name, config, deps), target_altitude_(50.0), takeoff_initiated_(false) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("alt", 50.0, "起飞高度(米)")
    };
  }
  BT::NodeStatus onStart();
  BT::NodeStatus onRunning();
  void onHalted();

 private:
  double target_altitude_;
  bool takeoff_initiated_;
  bool sendTakeoffCommand();
  bool checkAltitudeReached();
};