#pragma once

#include "behavior_node/base_nodes.hpp"

class LandAction : public StatefulActionBase<LandAction> {
 public:
  LandAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<LandAction>(name, config, deps), land_initiated_(false) {}

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:
  bool land_initiated_;
  bool sendLandCommand();

  bool checkLandingComplete();
};