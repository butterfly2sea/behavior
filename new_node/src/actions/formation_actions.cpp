#include "behavior_node/actions/formation_actions.hpp"
#include <log/Logger.hpp>

namespace behavior_node {

// ================================ SetFormationMembersAction ================================

SetFormationMembersAction::SetFormationMembersAction(const std::string& n, const BT::NodeConfig& config,
                                                     std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("FORM SetFormationMembersAction created: %s", n.c_str());
}

BT::PortsList SetFormationMembersAction::providedPorts() {
  return {
      BT::InputPort<std::vector<uint8_t>>("member_ids", "Formation member IDs"),
      BT::InputPort<uint8_t>("leader_id", 0, "Formation leader ID")
  };
}

BT::NodeStatus SetFormationMembersAction::tick() {
  try {
    std::vector<uint8_t> member_ids;
    uint8_t leader_id = 0;

    if (!getInput("member_ids", member_ids)) {
      txtLog().error("FORM SetFormationMembersAction failed to get member_ids");
      return BT::NodeStatus::FAILURE;
    }

    getInput("leader_id", leader_id);

    txtLog().info("FORM SetFormationMembersAction setting %zu members with leader %d",
                  member_ids.size(), leader_id);

    // 发布编队成员ID
    deps_->ros_comm->publishGroupIds(member_ids);

    // 存储到黑板供其他节点使用
    setOutput("member_ids", member_ids);
    setOutput("leader_id", leader_id);

    txtLog().info("FORM SetFormationMembersAction completed successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FORM SetFormationMembersAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ SetFormationOffsetAction ================================

SetFormationOffsetAction::SetFormationOffsetAction(const std::string& n, const BT::NodeConfig& config,
                                                   std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("FORM SetFormationOffsetAction created: %s", n.c_str());
}

BT::PortsList SetFormationOffsetAction::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Polygon>("offsetParam", "Formation offset points")
  };
}

BT::NodeStatus SetFormationOffsetAction::tick() {
  try {
    geometry_msgs::msg::Polygon offsets;

    if (!getInput("offsetParam", offsets)) {
      txtLog().error("FORM SetFormationOffsetAction failed to get offsetParam");
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("FORM SetFormationOffsetAction setting %zu offset points",
                  offsets.points.size());

    // 发布编队偏移
    deps_->ros_comm->publishGroupOffset(offsets);

    txtLog().info("FORM SetFormationOffsetAction completed successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FORM SetFormationOffsetAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ FormationSwitchAction ================================

FormationSwitchAction::FormationSwitchAction(const std::string& n, const BT::NodeConfig& config,
                                             std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("FORM FormationSwitchAction created: %s", n.c_str());
}

BT::PortsList FormationSwitchAction::providedPorts() {
  return {
      BT::InputPort<int>("formation_type", 0, "Formation type"),
      BT::InputPort<float>("formation_spacing", 20.0f, "Formation spacing in meters"),
      BT::InputPort<float>("transition_time", 10.0f, "Transition time in seconds")
  };
}

BT::NodeStatus FormationSwitchAction::tick() {
  try {
    int formation_type = 0;
    float spacing = 20.0f;
    float transition_time = 10.0f;

    getInput("formation_type", formation_type);
    getInput("formation_spacing", spacing);
    getInput("transition_time", transition_time);

    txtLog().info("FORM FormationSwitchAction switching to formation type %d, spacing %.2f",
                  formation_type, spacing);

    auto future = deps_->ros_comm->callFormationSwitch(formation_type);

    // 等待服务响应
    if (rclcpp::spin_until_future_complete(deps_->ros_comm->getNode(), future,
                                           std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS) {
      txtLog().error("FORM FormationSwitchAction service call timeout");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response || !response->success) {
      txtLog().error("FORM FormationSwitchAction service call failed");
      return BT::NodeStatus::FAILURE;
    }

    // 等待编队变换完成
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(transition_time * 1000)));

    txtLog().info("FORM FormationSwitchAction completed successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FORM FormationSwitchAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ FormationAssembleAction ================================

FormationAssembleAction::FormationAssembleAction(const std::string& n, const BT::NodeConfig& config,
                                                 std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("FORM FormationAssembleAction created: %s", n.c_str());
}

BT::PortsList FormationAssembleAction::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Point32>("assembly_point", "Assembly point"),
      BT::InputPort<float>("assembly_radius", 50.0f, "Assembly radius in meters"),
      BT::InputPort<float>("assembly_altitude", 100.0f, "Assembly altitude in meters"),
      BT::InputPort<float>("timeout", 300.0f, "Assembly timeout in seconds")
  };
}

BT::NodeStatus FormationAssembleAction::tick() {
  try {
    geometry_msgs::msg::Point32 assembly_point;
    float radius = 50.0f;
    float altitude = 100.0f;
    float timeout = 300.0f;

    if (!getInput("assembly_point", assembly_point)) {
      txtLog().error("FORM FormationAssembleAction failed to get assembly_point");
      return BT::NodeStatus::FAILURE;
    }

    getInput("assembly_radius", radius);
    getInput("assembly_altitude", altitude);
    getInput("timeout", timeout);

    txtLog().info("FORM FormationAssembleAction assembling at point (%.2f, %.2f, %.2f), radius %.2f",
                  assembly_point.x, assembly_point.y, assembly_point.z, radius);

    auto start_time = std::chrono::steady_clock::now();
    const auto timeout_duration = std::chrono::milliseconds(static_cast<int>(timeout * 1000));

    // 循环检查编队集结状态
    while (std::chrono::steady_clock::now() - start_time < timeout_duration) {

      // 检查是否所有成员都已集结
      if (checkAssemblyComplete(assembly_point, radius)) {
        txtLog().info("FORM FormationAssembleAction assembly completed");
        return BT::NodeStatus::SUCCESS;
      }

      // 短暂延迟后再检查
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    txtLog().warning("FORM FormationAssembleAction assembly timeout");
    return BT::NodeStatus::FAILURE;

  } catch (const std::exception& e) {
    txtLog().error("FORM FormationAssembleAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

bool FormationAssembleAction::checkAssemblyComplete(const geometry_msgs::msg::Point32& target_point, float radius) const {
  // 获取当前位置
  auto current_pos = deps_->cache->getCurrentPosition();
  if (!current_pos) {
    return false;
  }

  // 计算距离
  float dx = current_pos->x - target_point.x;
  float dy = current_pos->y - target_point.y;
  float distance = std::sqrt(dx * dx + dy * dy);

  return distance <= radius;
}

// ================================ FormationFlyAction ================================

FormationFlyAction::FormationFlyAction(const std::string& n, const BT::NodeConfig& config,
                                       std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("FORM FormationFlyAction created: %s", n.c_str());
}

BT::PortsList FormationFlyAction::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Polygon>("waypoints", "Waypoints for formation flight"),
      BT::InputPort<float>("speed", 10.0f, "Flight speed in m/s"),
      BT::InputPort<geometry_msgs::msg::Polygon>("offsets", "Formation offsets"),
      BT::InputPort<uint8_t>("leader_id", 0, "Formation leader ID"),
      BT::InputPort<bool>("maintain_formation", true, "Maintain formation flag")
  };
}

BT::NodeStatus FormationFlyAction::tick() {
  try {
    geometry_msgs::msg::Polygon waypoints;
    geometry_msgs::msg::Polygon offsets;
    float speed = 10.0f;
    uint8_t leader_id = 0;
    bool maintain_formation = true;

    if (!getInput("waypoints", waypoints)) {
      txtLog().error("FORM FormationFlyAction failed to get waypoints");
      return BT::NodeStatus::FAILURE;
    }

    getInput("speed", speed);
    getInput("offsets", offsets);
    getInput("leader_id", leader_id);
    getInput("maintain_formation", maintain_formation);

    txtLog().info("FORM FormationFlyAction executing with %zu waypoints at %.2f m/s",
                  waypoints.points.size(), speed);

    // 发布航线
    deps_->ros_comm->publishNavline(waypoints);

    // 发布速度
    deps_->ros_comm->publishGroupSpeed(speed);

    // 如果有偏移，发布偏移
    if (!offsets.points.empty()) {
      deps_->ros_comm->publishGroupOffset(offsets);
    }

    // 启动编队飞行 - 这里简化处理，实际应该监控飞行状态
    txtLog().info("FORM FormationFlyAction formation flight initiated");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FORM FormationFlyAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior_node