#include "behavior_node/conditions/search_conditions.hpp"
#include <log/Logger.hpp>

namespace behavior_node {

// ================================ CheckTargetFoundCondition ================================

CheckTargetFoundCondition::CheckTargetFoundCondition(const std::string& n, const BT::NodeConfig& config,
                                                     std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("SEARCH CheckTargetFoundCondition created: %s", n.c_str());
}

BT::PortsList CheckTargetFoundCondition::providedPorts() {
  return {
      BT::InputPort<int>("target_class", -1, "Target class ID (-1 for any)"),
      BT::InputPort<float>("confidence_threshold", 0.7f, "Minimum confidence threshold"),
      BT::InputPort<float>("max_age_seconds", 5.0f, "Maximum target age in seconds")
  };
}

BT::NodeStatus CheckTargetFoundCondition::tick() {
  try {
    int target_class = -1;
    float confidence_threshold = 0.7f;
    float max_age = 5.0f;

    getInput("target_class", target_class);
    getInput("confidence_threshold", confidence_threshold);
    getInput("max_age_seconds", max_age);

    auto detected_targets = deps_->cache->getDetectedTargets();
    if (!detected_targets || detected_targets->empty()) {
      txtLog().debug("SEARCH CheckTargetFoundCondition no targets detected");
      return BT::NodeStatus::FAILURE;
    }

    auto now = std::chrono::steady_clock::now();

    for (const auto& target : *detected_targets) {
      // 检查目标类别
      if (target_class >= 0 && target.class_id != target_class) {
        continue;
      }

      // 检查置信度
      if (target.confidence < confidence_threshold) {
        continue;
      }

      // 检查目标年龄
      auto target_age = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - target.detection_time).count() / 1000.0f;

      if (target_age > max_age) {
        continue;
      }

      txtLog().info("SEARCH CheckTargetFoundCondition target found: class=%d, conf=%.2f, age=%.1fs",
                    target.class_id, target.confidence, target_age);

      // 将找到的目标信息输出到黑板
      setOutput("found_target_id", target.target_id);
      setOutput("found_target_class", target.class_id);
      setOutput("found_target_confidence", target.confidence);
      setOutput("found_target_position", target.position);

      return BT::NodeStatus::SUCCESS;
    }

    txtLog().debug("SEARCH CheckTargetFoundCondition no valid targets found");
    return BT::NodeStatus::FAILURE;

  } catch (const std::exception& e) {
    txtLog().error("SEARCH CheckTargetFoundCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckSearchAreaCompleteCondition ================================

CheckSearchAreaCompleteCondition::CheckSearchAreaCompleteCondition(const std::string& n, const BT::NodeConfig& config,
                                                                   std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("SEARCH CheckSearchAreaCompleteCondition created: %s", n.c_str());
}

BT::PortsList CheckSearchAreaCompleteCondition::providedPorts() {
  return {
      BT::InputPort<geometry_msgs::msg::Polygon>("search_area", "Search area polygon"),
      BT::InputPort<float>("coverage_threshold", 0.8f, "Coverage completion threshold (0.0-1.0)")
  };
}

BT::NodeStatus CheckSearchAreaCompleteCondition::tick() {
  try {
    geometry_msgs::msg::Polygon search_area;
    float coverage_threshold = 0.8f;

    if (!getInput("search_area", search_area)) {
      txtLog().error("SEARCH CheckSearchAreaCompleteCondition failed to get search_area");
      return BT::NodeStatus::FAILURE;
    }

    getInput("coverage_threshold", coverage_threshold);

    auto search_status = deps_->cache->getSearchStatus();
    if (!search_status) {
      txtLog().warning("SEARCH CheckSearchAreaCompleteCondition no search status");
      return BT::NodeStatus::FAILURE;
    }

    float coverage_percentage = search_status->coverage_percentage;

    if (coverage_percentage >= coverage_threshold) {
      txtLog().info("SEARCH CheckSearchAreaCompleteCondition search area complete: %.1f%% >= %.1f%%",
                    coverage_percentage * 100.0f, coverage_threshold * 100.0f);
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().debug("SEARCH CheckSearchAreaCompleteCondition search area not complete: %.1f%% < %.1f%%",
                     coverage_percentage * 100.0f, coverage_threshold * 100.0f);
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("SEARCH CheckSearchAreaCompleteCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckSearchTimeoutCondition ================================

CheckSearchTimeoutCondition::CheckSearchTimeoutCondition(const std::string& n, const BT::NodeConfig& config,
                                                         std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps), search_start_time_(std::chrono::steady_clock::now()) {
  txtLog().debug("SEARCH CheckSearchTimeoutCondition created: %s", n.c_str());
}

BT::PortsList CheckSearchTimeoutCondition::providedPorts() {
  return {
      BT::InputPort<float>("max_search_time", 600.0f, "Maximum search time in seconds")
  };
}

BT::NodeStatus CheckSearchTimeoutCondition::tick() {
  try {
    float max_search_time = 600.0f;
    getInput("max_search_time", max_search_time);

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - search_start_time_).count();

    if (elapsed >= static_cast<int64_t>(max_search_time)) {
      txtLog().info("SEARCH CheckSearchTimeoutCondition search timeout: %lds >= %.1fs",
                    elapsed, max_search_time);
      return BT::NodeStatus::SUCCESS;  // 超时返回SUCCESS表示条件满足
    } else {
      txtLog().debug("SEARCH CheckSearchTimeoutCondition search ongoing: %lds < %.1fs",
                     elapsed, max_search_time);
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("SEARCH CheckSearchTimeoutCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void CheckSearchTimeoutCondition::resetTimer() {
  search_start_time_ = std::chrono::steady_clock::now();
  txtLog().debug("SEARCH CheckSearchTimeoutCondition timer reset");
}

// ================================ CheckQuitSearchCondition ================================

CheckQuitSearchCondition::CheckQuitSearchCondition(const std::string& n, const BT::NodeConfig& config,
                                                   std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("SEARCH CheckQuitSearchCondition created: %s", n.c_str());
}

BT::PortsList CheckQuitSearchCondition::providedPorts() {
  return {
      BT::InputPort<float>("max_search_time", 600.0f, "Maximum search time in seconds"),
      BT::InputPort<bool>("quit_on_target_found", true, "Quit search when target is found"),
      BT::InputPort<int>("target_class", -1, "Target class to look for (-1 for any)"),
      BT::InputPort<float>("confidence_threshold", 0.7f, "Minimum confidence for target detection")
  };
}

BT::NodeStatus CheckQuitSearchCondition::tick() {
  try {
    float max_search_time = 600.0f;
    bool quit_on_target = true;
    int target_class = -1;
    float confidence_threshold = 0.7f;

    getInput("max_search_time", max_search_time);
    getInput("quit_on_target_found", quit_on_target);
    getInput("target_class", target_class);
    getInput("confidence_threshold", confidence_threshold);

    // 检查是否超时
    auto search_status = deps_->cache->getSearchStatus();
    if (search_status) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
          now - search_status->start_time).count();

      if (elapsed >= static_cast<int64_t>(max_search_time)) {
        txtLog().info("SEARCH CheckQuitSearchCondition quitting due to timeout: %lds", elapsed);
        return BT::NodeStatus::SUCCESS;
      }
    }

    // 检查是否找到目标
    if (quit_on_target) {
      auto detected_targets = deps_->cache->getDetectedTargets();
      if (detected_targets && !detected_targets->empty()) {

        for (const auto& target : *detected_targets) {
          // 检查目标类别
          if (target_class >= 0 && target.class_id != target_class) {
            continue;
          }

          // 检查置信度
          if (target.confidence >= confidence_threshold) {
            txtLog().info("SEARCH CheckQuitSearchCondition quitting due to target found: class=%d, conf=%.2f",
                          target.class_id, target.confidence);
            return BT::NodeStatus::SUCCESS;
          }
        }
      }
    }

    // 检查外部退出命令
    if (auto mission_context = deps_->mission_context) {
      if (mission_context->getParameter("quit_search").has_value()) {
        txtLog().info("SEARCH CheckQuitSearchCondition quitting due to external command");
        return BT::NodeStatus::SUCCESS;
      }
    }

    txtLog().debug("SEARCH CheckQuitSearchCondition continue searching");
    return BT::NodeStatus::FAILURE;

  } catch (const std::exception& e) {
    txtLog().error("SEARCH CheckQuitSearchCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckTargetInRangeCondition ================================

CheckTargetInRangeCondition::CheckTargetInRangeCondition(const std::string& n, const BT::NodeConfig& config,
                                                         std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("SEARCH CheckTargetInRangeCondition created: %s", n.c_str());
}

BT::PortsList CheckTargetInRangeCondition::providedPorts() {
  return {
      BT::InputPort<int32_t>("target_id", -1, "Target ID to check"),
      BT::InputPort<float>("max_range", 100.0f, "Maximum range in meters"),
      BT::InputPort<float>("min_range", 0.0f, "Minimum range in meters")
  };
}

BT::NodeStatus CheckTargetInRangeCondition::tick() {
  try {
    int32_t target_id = -1;
    float max_range = 100.0f;
    float min_range = 0.0f;

    getInput("target_id", target_id);
    getInput("max_range", max_range);
    getInput("min_range", min_range);

    auto current_pos = deps_->cache->getCurrentPosition();
    if (!current_pos) {
      txtLog().warning("SEARCH CheckTargetInRangeCondition no current position");
      return BT::NodeStatus::FAILURE;
    }

    auto detected_targets = deps_->cache->getDetectedTargets();
    if (!detected_targets || detected_targets->empty()) {
      txtLog().debug("SEARCH CheckTargetInRangeCondition no targets detected");
      return BT::NodeStatus::FAILURE;
    }

    for (const auto& target : *detected_targets) {
      if (target_id >= 0 && target.target_id != target_id) {
        continue;
      }

      // 计算距离
      float dx = current_pos->x - target.position.x;
      float dy = current_pos->y - target.position.y;
      float dz = current_pos->z - target.position.z;
      float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      bool in_range = (distance >= min_range && distance <= max_range);

      if (in_range) {
        txtLog().info("SEARCH CheckTargetInRangeCondition target %d in range: %.2fm [%.2f, %.2f]",
                      target.target_id, distance, min_range, max_range);

        // 输出目标信息
        setOutput("range_to_target", distance);
        setOutput("target_position", target.position);

        return BT::NodeStatus::SUCCESS;
      }
    }

    txtLog().debug("SEARCH CheckTargetInRangeCondition no targets in range");
    return BT::NodeStatus::FAILURE;

  } catch (const std::exception& e) {
    txtLog().error("SEARCH CheckTargetInRangeCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior_node