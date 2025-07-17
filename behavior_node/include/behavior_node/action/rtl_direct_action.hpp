#pragma once

#include "behavior_node/base_nodes.hpp"

class RtlDirectAction : public StatefulActionBase<RtlDirectAction> {
 public:
  RtlDirectAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<RtlDirectAction>(name, config, deps), rtl_initiated_(false) {}

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

 private:
  bool rtl_initiated_;
  std::chrono::steady_clock::time_point hover_start_;
  bool sendRtlCommand();
  bool checkRtlComplete();

};