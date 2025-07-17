#pragma once

#include "behavior_node/base_nodes.hpp"

class AttackAction : public StatefulActionBase<AttackAction> {

 public:
  AttackAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<AttackAction>(name, config, deps),
        target_id_(0),
        src_id_(0),
        dst_id_(0),
        attack_initiated_(false) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("tgtId", "目标ID"),
        BT::InputPort<int>("srcId", "源ID"),
        BT::InputPort<int>("dstId", "目标ID")
    };
  }

  BT::NodeStatus onStart() override ;

  BT::NodeStatus onRunning() override;
  void onHalted() override ;

 private:
  int target_id_;
  int src_id_;
  int dst_id_;
  bool attack_initiated_;
  bool loadAttackParameters() ;

  bool sendAttackCommand() ;

  bool checkAttackComplete() ;
};