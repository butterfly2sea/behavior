#include "behavior_node/action/attack_action.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"

BT::NodeStatus AttackAction::onStart() {
// 加载攻击参数
  if (!loadAttackParameters()) {
    return BT::NodeStatus::FAILURE;
  }

  txtLog().info(THISMODULE "Starting attack on target %d", target_id_);

  if (!sendAttackCommand()) {
    return BT::NodeStatus::FAILURE;
  }

  attack_initiated_ = true;
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AttackAction::onRunning() {
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > timeout_) {
    txtLog().error(THISMODULE "Attack timeout");
    return BT::NodeStatus::FAILURE;
  }

// 检查攻击是否完成
  if (checkAttackComplete()) {
    txtLog().info(THISMODULE "Attack completed successfully");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void AttackAction::onHalted() {
  attack_initiated_ = false;
  txtLog().info(THISMODULE "Attack action halted");
}

bool AttackAction::loadAttackParameters() {
  if (!context()->hasParameter("tgtId") || !context()->hasParameter("srcId") || !context()->hasParameter("dstId")) {
    txtLog().error(THISMODULE "Missing attack parameters");
    return false;
  }
  auto tgt_param = context()->getParameter("tgtId");
  auto src_param = context()->getParameter("srcId");
  auto dst_param = context()->getParameter("dstId");

  target_id_ = std::stoi(tgt_param.get<std::string>());
  src_id_ = std::stoi(src_param.get<std::string>());
  dst_id_ = std::stoi(dst_param.get<std::string>());

  return target_id_ > 0;
}

bool AttackAction::sendAttackCommand() {
  try {
    return ros()->callAttackService(target_id_, src_id_, dst_id_);
  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Failed to send attack command: %s", e.what());
    return false;
  }
}

bool AttackAction::checkAttackComplete() {
  // 这里应该检查攻击任务的完成状态
  // 具体实现取决于系统的攻击状态反馈机制
  return false; // 简化实现
}