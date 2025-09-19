#include "behavior_node/conditions/navigation_conditions.hpp"
#include <log/Logger.hpp>
#include <cmath>

namespace behavior_node {

// ================================ CheckWaypointReachedCondition ================================

CheckWaypointReachedCondition::CheckWaypointReachedCondition(const std::string& n, const BT::NodeConfig& config,
                                                             std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("NAV CheckWaypointReachedCondition created: %s", n.c_str());
}

BT::PortsList CheckWaypointReachedCondition::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Point32>("target_waypoint", "Target waypoint"),
      BT::InputPort<float>("arrival_distance", 5.0f, "Arrival distance threshold in meters"),
      BT::InputPort<bool>("check_altitude", false, "Whether to check altitude")
  };
}

BT::NodeStatus CheckWaypointReachedCondition::tick() {
  try {
    geometry_msgs::msg::Point32 target;
    float arrival_dist = 5.0f;
    bool check_altitude = false;

    if (!getInput("target_waypoint", target)) {
      txtLog().error("NAV CheckWaypointReachedCondition failed to get target_waypoint");
      return BT::NodeStatus::FAILURE;
    }

    getInput("arrival_distance", arrival_dist);
    getInput("check_altitude", check_altitude);

    auto current_pos = deps_->cache->getCurrentPosition();
    if (!current_pos) {
      txtLog().warning("NAV CheckWaypointReachedCondition no current position");
      return BT::NodeStatus::FAILURE;
    }

    // 计算水平距离
    float dx = current_pos->x - target.x;
    float dy = current_pos->y - target.y;
    float horizontal_dist = std::sqrt(dx * dx + dy * dy);

    bool reached = horizontal_dist <= arrival_dist;

    // 如果需要检查高度
    if (check_altitude && reached) {
      float dz = std::abs(current_pos->z - target.z);
      reached = reached && (dz <= arrival_dist);
    }

    if (reached) {
      txtLog().info("NAV CheckWaypointReachedCondition waypoint reached: dist=%.2f <= %.2f",
                    horizontal_dist, arrival_dist);
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().debug("NAV CheckWaypointReachedCondition waypoint not reached: dist=%.2f > %.2f",
                     horizontal_dist, arrival_dist);
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("NAV CheckWaypointReachedCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckRouteCompletedCondition ================================

CheckRouteCompletedCondition::CheckRouteCompletedCondition(const std::string& n, const BT::NodeConfig& config,
                                                           std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("NAV CheckRouteCompletedCondition created: %s", n.c_str());
}

BT::PortsList CheckRouteCompletedCondition::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Polygon>("waypoints", "Route waypoints"),
      BT::InputPort<float>("arrival_distance", 5.0f, "Arrival distance threshold"),
      BT::InputPort<int>("required_loops", 1, "Required number of loops (-1 for infinite)")
  };
}

BT::NodeStatus CheckRouteCompletedCondition::tick() {
  try {
    geometry_msgs::msg::Polygon waypoints;
    float arrival_dist = 5.0f;
    int required_loops = 1;

    if (!getInput("waypoints", waypoints)) {
      txtLog().error("NAV CheckRouteCompletedCondition failed to get waypoints");
      return BT::NodeStatus::FAILURE;
    }

    getInput("arrival_distance", arrival_dist);
    getInput("required_loops", required_loops);

    if (waypoints.points.empty()) {
      txtLog().warning("NAV CheckRouteCompletedCondition empty waypoints");
      return BT::NodeStatus::SUCCESS;  // 空航线认为已完成
    }

    // 获取导航状态
    auto nav_status = deps_->cache->getNavigationStatus();
    if (!nav_status) {
      txtLog().warning("NAV CheckRouteCompletedCondition no navigation status");
      return BT::NodeStatus::FAILURE;
    }

    // 检查是否完成了所需的循环次数
    if (required_loops > 0 && nav_status->completed_loops >= required_loops) {
      txtLog().info("NAV CheckRouteCompletedCondition route completed: %d/%d loops",
                    nav_status->completed_loops, required_loops);
      return BT::NodeStatus::SUCCESS;
    }

    // 检查是否到达最后一个航点
    auto current_pos = deps_->cache->getCurrentPosition();
    if (!current_pos) {
      return BT::NodeStatus::FAILURE;
    }

    const auto& last_waypoint = waypoints.points.back();
    float dx = current_pos->x - last_waypoint.x;
    float dy = current_pos->y - last_waypoint.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    bool at_last_waypoint = dist <= arrival_dist;

    if (required_loops < 0) {
      // 无限循环，永远不完成
      return BT::NodeStatus::FAILURE;
    } else if (at_last_waypoint && nav_status->completed_loops >= required_loops) {
      return BT::NodeStatus::SUCCESS;
    }

    txtLog().debug("NAV CheckRouteCompletedCondition route not completed: loops=%d/%d, dist_to_last=%.2f",
                   nav_status->completed_loops, required_loops, dist);
    return BT::NodeStatus::FAILURE;

  } catch (const std::exception& e) {
    txtLog().error("NAV CheckRouteCompletedCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckNavigationActiveCondition ================================

CheckNavigationActiveCondition::CheckNavigationActiveCondition(const std::string& n, const BT::NodeConfig& config,
                                                               std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("NAV CheckNavigationActiveCondition created: %s", n.c_str());
}

BT::PortsList CheckNavigationActiveCondition::providedPorts() {
  return {};  // 无需输入参数
}

BT::NodeStatus CheckNavigationActiveCondition::tick() {
  try {
    auto nav_status = deps_->cache->getNavigationStatus();
    if (!nav_status) {
      txtLog().debug("NAV CheckNavigationActiveCondition no navigation status");
      return BT::NodeStatus::FAILURE;
    }

    if (nav_status->is_active) {
      txtLog().debug("NAV CheckNavigationActiveCondition navigation is active");
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().debug("NAV CheckNavigationActiveCondition navigation is not active");
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("NAV CheckNavigationActiveCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckDistanceToTargetCondition ================================

CheckDistanceToTargetCondition::CheckDistanceToTargetCondition(const std::string& n, const BT::NodeConfig& config,
                                                               std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("NAV CheckDistanceToTargetCondition created: %s", n.c_str());
}

BT::PortsList CheckDistanceToTargetCondition::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Point32>("target_point", "Target point"),
      BT::InputPort<float>("min_distance", 0.0f, "Minimum distance threshold"),
      BT::InputPort<float>("max_distance", 1000.0f, "Maximum distance threshold"),
      BT::InputPort<std::string>("comparison", "WITHIN", "Comparison type: WITHIN, LESS_THAN, GREATER_THAN")
  };
}

BT::NodeStatus CheckDistanceToTargetCondition::tick() {
  try {
    geometry_msgs::msg::Point32 target;
    float min_dist = 0.0f;
    float max_dist = 1000.0f;
    std::string comparison = "WITHIN";

    if (!getInput("target_point", target)) {
      txtLog().error("NAV CheckDistanceToTargetCondition failed to get target_point");
      return BT::NodeStatus::FAILURE;
    }

    getInput("min_distance", min_dist);
    getInput("max_distance", max_dist);
    getInput("comparison", comparison);

    auto current_pos = deps_->cache->getCurrentPosition();
    if (!current_pos) {
      txtLog().warning("NAV CheckDistanceToTargetCondition no current position");
      return BT::NodeStatus::FAILURE;
    }

    // 计算距离
    float dx = current_pos->x - target.x;
    float dy = current_pos->y - target.y;
    float dz = current_pos->z - target.z;
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    bool condition_met = false;

    if (comparison == "WITHIN") {
      condition_met = (distance >= min_dist && distance <= max_dist);
    } else if (comparison == "LESS_THAN") {
      condition_met = (distance < max_dist);
    } else if (comparison == "GREATER_THAN") {
      condition_met = (distance > min_dist);
    } else {
      txtLog().error("NAV CheckDistanceToTargetCondition invalid comparison: %s", comparison.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (condition_met) {
      txtLog().debug("NAV CheckDistanceToTargetCondition condition met: %.2f %s [%.2f, %.2f]",
                     distance, comparison.c_str(), min_dist, max_dist);
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().debug("NAV CheckDistanceToTargetCondition condition not met: %.2f %s [%.2f, %.2f]",
                     distance, comparison.c_str(), min_dist, max_dist);
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("NAV CheckDistanceToTargetCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckHeadingCondition ================================

CheckHeadingCondition::CheckHeadingCondition(const std::string& n, const BT::NodeConfig& config,
                                             std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("NAV CheckHeadingCondition created: %s", n.c_str());
}

BT::PortsList CheckHeadingCondition::providedPorts() {
  return {
      BT::InputPort<float>("target_heading", 0.0f, "Target heading in degrees"),
      BT::InputPort<float>("tolerance", 5.0f, "Heading tolerance in degrees")
  };
}

BT::NodeStatus CheckHeadingCondition::tick() {
  try {
    float target_heading = 0.0f;
    float tolerance = 5.0f;

    getInput("target_heading", target_heading);
    getInput("tolerance", tolerance);

    auto attitude = deps_->cache->getCurrentAttitude();
    if (!attitude) {
      txtLog().warning("NAV CheckHeadingCondition no attitude available");
      return BT::NodeStatus::FAILURE;
    }

    // 将四元数转换为欧拉角获取偏航角
    float yaw = quatToYaw(attitude->x, attitude->y, attitude->z, attitude->w);

    // 转换为度数
    float current_heading = yaw * 180.0f / M_PI;

    // 归一化角度到 [0, 360)
    current_heading = fmod(current_heading + 360.0f, 360.0f);
    target_heading = fmod(target_heading + 360.0f, 360.0f);

    // 计算角度差
    float heading_diff = std::abs(current_heading - target_heading);
    if (heading_diff > 180.0f) {
      heading_diff = 360.0f - heading_diff;
    }

    bool heading_ok = heading_diff <= tolerance;

    if (heading_ok) {
      txtLog().debug("NAV CheckHeadingCondition heading OK: %.1f° vs %.1f° (diff=%.1f°)",
                     current_heading, target_heading, heading_diff);
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().debug("NAV CheckHeadingCondition heading not OK: %.1f° vs %.1f° (diff=%.1f°)",
                     current_heading, target_heading, heading_diff);
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("NAV CheckHeadingCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

float CheckHeadingCondition::quatToYaw(float x, float y, float z, float w) const {
  // 从四元数计算偏航角
  return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

}  // namespace behavior_node