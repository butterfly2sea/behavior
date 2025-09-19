#include "behavior_node/conditions/system_conditions.hpp"
#include <log/Logger.hpp>

namespace behavior_node {

// ================================ CheckSystemHealthyCondition ================================

CheckSystemHealthyCondition::CheckSystemHealthyCondition(const std::string& n, const BT::NodeConfig& config,
                                                         std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("SYS CheckSystemHealthyCondition created: %s", n.c_str());
}

BT::PortsList CheckSystemHealthyCondition::providedPorts() {
  return {
      BT::InputPort<bool>("check_sensors", true, "Check sensor health"),
      BT::InputPort<bool>("check_communication", true, "Check communication health"),
      BT::InputPort<bool>("check_power", true, "Check power system health"),
      BT::InputPort<float>("min_battery_percentage", 15.0f, "Minimum battery percentage for healthy")
  };
}

BT::NodeStatus CheckSystemHealthyCondition::tick() {
  try {
    bool check_sensors = true;
    bool check_comm = true;
    bool check_power = true;
    float min_battery = 15.0f;

    getInput("check_sensors", check_sensors);
    getInput("check_communication", check_comm);
    getInput("check_power", check_power);
    getInput("min_battery_percentage", min_battery);

    // 检查传感器健康状态
    if (check_sensors) {
      auto health_status = deps_->cache->getHealthStatus();
      if (!health_status || !health_status->sensors_healthy) {
        txtLog().warning("SYS CheckSystemHealthyCondition sensors not healthy");
        return BT::NodeStatus::FAILURE;
      }
    }

    // 检查通信健康状态
    if (check_comm) {
      auto comm_status = deps_->cache->getCommunicationStatus();
      if (!comm_status || !comm_status->is_connected) {
        txtLog().warning("SYS CheckSystemHealthyCondition communication not healthy");
        return BT::NodeStatus::FAILURE;
      }
    }

    // 检查电源系统
    if (check_power) {
      auto battery_state = deps_->cache->getBatteryState();
      if (!battery_state) {
        txtLog().warning("SYS CheckSystemHealthyCondition no battery state");
        return BT::NodeStatus::FAILURE;
      }

      if (battery_state->percentage < min_battery) {
        txtLog().warning("SYS CheckSystemHealthyCondition battery level too low: %.1f%% < %.1f%%",
                         battery_state->percentage, min_battery);
        return BT::NodeStatus::FAILURE;
      }
    }

    txtLog().debug("SYS CheckSystemHealthyCondition system healthy");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("SYS CheckSystemHealthyCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckEmergencyCondition ================================

CheckEmergencyCondition::CheckEmergencyCondition(const std::string& n, const BT::NodeConfig& config,
                                                 std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("SYS CheckEmergencyCondition created: %s", n.c_str());
}

BT::PortsList CheckEmergencyCondition::providedPorts() {
  return {
      BT::InputPort<float>("critical_battery_level", 10.0f, "Critical battery level percentage"),
      BT::InputPort<bool>("check_geofence", true, "Check geofence violations"),
      BT::InputPort<bool>("check_communication_loss", true, "Check communication loss")
  };
}

BT::NodeStatus CheckEmergencyCondition::tick() {
  try {
    float critical_battery = 10.0f;
    bool check_geofence = true;
    bool check_comm_loss = true;

    getInput("critical_battery_level", critical_battery);
    getInput("check_geofence", check_geofence);
    getInput("check_communication_loss", check_comm_loss);

    // 检查电池危险电量
    auto battery_state = deps_->cache->getBatteryState();
    if (battery_state && battery_state->percentage <= critical_battery) {
      txtLog().warning("SYS CheckEmergencyCondition critical battery level: %.1f%%",
                       battery_state->percentage);
      return BT::NodeStatus::SUCCESS;  // 紧急情况返回SUCCESS
    }

    // 检查地理围栏违规
    if (check_geofence) {
      auto geofence_status = deps_->cache->getGeofenceStatus();
      if (geofence_status && geofence_status->violation_detected) {
        txtLog().warning("SYS CheckEmergencyCondition geofence violation detected");
        return BT::NodeStatus::SUCCESS;
      }
    }

    // 检查通信丢失
    if (check_comm_loss) {
      auto comm_status = deps_->cache->getCommunicationStatus();
      if (comm_status && comm_status->connection_lost) {
        txtLog().warning("SYS CheckEmergencyCondition communication lost");
        return BT::NodeStatus::SUCCESS;
      }
    }

    // 检查手动紧急停止命令
    if (auto mission_context = deps_->mission_context) {
      if (mission_context->getParameter("emergency_stop").has_value()) {
        txtLog().warning("SYS CheckEmergencyCondition manual emergency stop triggered");
        return BT::NodeStatus::SUCCESS;
      }
    }

    txtLog().debug("SYS CheckEmergencyCondition no emergency detected");
    return BT::NodeStatus::FAILURE;

  } catch (const std::exception& e) {
    txtLog().error("SYS CheckEmergencyCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckMissionParameterCondition ================================

CheckMissionParameterCondition::CheckMissionParameterCondition(const std::string& n, const BT::NodeConfig& config,
                                                               std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("SYS CheckMissionParameterCondition created: %s", n.c_str());
}

BT::PortsList CheckMissionParameterCondition::providedPorts() {
  return {
      BT::InputPort<std::string>("parameter_name", "Parameter name to check"),
      BT::InputPort<std::string>("expected_value", "", "Expected parameter value"),
      BT::InputPort<std::string>("comparison_type", "EQUALS", "Comparison type: EQUALS, NOT_EQUALS, EXISTS, NOT_EXISTS")
  };
}

BT::NodeStatus CheckMissionParameterCondition::tick() {
  try {
    std::string param_name;
    std::string expected_value;
    std::string comparison = "EQUALS";

    if (!getInput("parameter_name", param_name)) {
      txtLog().error("SYS CheckMissionParameterCondition failed to get parameter_name");
      return BT::NodeStatus::FAILURE;
    }

    getInput("expected_value", expected_value);
    getInput("comparison_type", comparison);

    if (!deps_->mission_context) {
      txtLog().warning("SYS CheckMissionParameterCondition no mission context");
      return BT::NodeStatus::FAILURE;
    }

    auto param_value = deps_->mission_context->getParameter(param_name);
    bool has_parameter = param_value.has_value();

    bool condition_met = false;

    if (comparison == "EXISTS") {
      condition_met = has_parameter;
    } else if (comparison == "NOT_EXISTS") {
      condition_met = !has_parameter;
    } else if (comparison == "EQUALS") {
      if (has_parameter) {
        // 将参数值转换为字符串进行比较
        std::string param_str = param_value->dump();
        // 移除JSON字符串的引号
        if (param_str.front() == '"' && param_str.back() == '"') {
          param_str = param_str.substr(1, param_str.length() - 2);
        }
        condition_met = (param_str == expected_value);
      } else {
        condition_met = false;
      }
    } else if (comparison == "NOT_EQUALS") {
      if (has_parameter) {
        std::string param_str = param_value->dump();
        if (param_str.front() == '"' && param_str.back() == '"') {
          param_str = param_str.substr(1, param_str.length() - 2);
        }
        condition_met = (param_str != expected_value);
      } else {
        condition_met = true;  // 不存在的参数认为不等于期望值
      }
    } else {
      txtLog().error("SYS CheckMissionParameterCondition invalid comparison type: %s", comparison.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (condition_met) {
      txtLog().debug("SYS CheckMissionParameterCondition parameter check passed: %s %s %s",
                     param_name.c_str(), comparison.c_str(), expected_value.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().debug("SYS CheckMissionParameterCondition parameter check failed: %s %s %s",
                     param_name.c_str(), comparison.c_str(), expected_value.c_str());
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("SYS CheckMissionParameterCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckTimeCondition ================================

CheckTimeCondition::CheckTimeCondition(const std::string& n, const BT::NodeConfig& config,
                                       std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps), start_time_(std::chrono::steady_clock::now()) {
  txtLog().debug("SYS CheckTimeCondition created: %s", n.c_str());
}

BT::PortsList CheckTimeCondition::providedPorts() {
  return {
      BT::InputPort<float>("duration_seconds", 10.0f, "Duration in seconds"),
      BT::InputPort<std::string>("comparison", "ELAPSED", "Comparison type: ELAPSED, REMAINING"),
      BT::InputPort<bool>("reset_on_tick", false, "Reset timer on each tick")
  };
}

BT::NodeStatus CheckTimeCondition::tick() {
  try {
    float duration = 10.0f;
    std::string comparison = "ELAPSED";
    bool reset_on_tick = false;

    getInput("duration_seconds", duration);
    getInput("comparison", comparison);
    getInput("reset_on_tick", reset_on_tick);

    if (reset_on_tick) {
      start_time_ = std::chrono::steady_clock::now();
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count() / 1000.0f;

    bool condition_met = false;

    if (comparison == "ELAPSED") {
      condition_met = (elapsed >= duration);
    } else if (comparison == "REMAINING") {
      condition_met = (elapsed < duration);
    } else {
      txtLog().error("SYS CheckTimeCondition invalid comparison: %s", comparison.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (condition_met) {
      txtLog().debug("SYS CheckTimeCondition time condition met: %.1fs %s %.1fs",
                     elapsed, comparison.c_str(), duration);
      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().debug("SYS CheckTimeCondition time condition not met: %.1fs %s %.1fs",
                     elapsed, comparison.c_str(), duration);
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("SYS CheckTimeCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

void CheckTimeCondition::resetTimer() {
  start_time_ = std::chrono::steady_clock::now();
  txtLog().debug("SYS CheckTimeCondition timer reset");
}

// ================================ CheckGroundStationCommandCondition ================================

CheckGroundStationCommandCondition::CheckGroundStationCommandCondition(const std::string& n, const BT::NodeConfig& config,
                                                                       std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("SYS CheckGroundStationCommandCondition created: %s", n.c_str());
}

BT::PortsList CheckGroundStationCommandCondition::providedPorts() {
  return {
      BT::InputPort<std::string>("expected_command", "Command to wait for"),
      BT::InputPort<float>("timeout_seconds", 0.0f, "Timeout in seconds (0 for no timeout)"),
      BT::InputPort<bool>("consume_command", true, "Whether to consume the command after checking")
  };
}

BT::NodeStatus CheckGroundStationCommandCondition::tick() {
  try {
    std::string expected_command;
    float timeout = 0.0f;
    bool consume = true;

    if (!getInput("expected_command", expected_command)) {
      txtLog().error("SYS CheckGroundStationCommandCondition failed to get expected_command");
      return BT::NodeStatus::FAILURE;
    }

    getInput("timeout_seconds", timeout);
    getInput("consume_command", consume);

    if (!deps_->mission_context) {
      txtLog().warning("SYS CheckGroundStationCommandCondition no mission context");
      return BT::NodeStatus::FAILURE;
    }

    // 检查是否有匹配的地面站命令
    auto gs_command = deps_->mission_context->getGroundStationCommand();
    if (!gs_command.has_value()) {
      txtLog().debug("SYS CheckGroundStationCommandCondition no ground station command");
      return BT::NodeStatus::FAILURE;
    }

    std::string command_str = gs_command->dump();
    // 移除JSON字符串的引号
    if (command_str.front() == '"' && command_str.back() == '"') {
      command_str = command_str.substr(1, command_str.length() - 2);
    }

    bool command_matched = (command_str == expected_command);

    if (command_matched) {
      txtLog().info("SYS CheckGroundStationCommandCondition command matched: %s",
                    expected_command.c_str());

      // 如果设置消费命令，清除该命令
      if (consume) {
        deps_->mission_context->clearGroundStationCommand();
      }

      return BT::NodeStatus::SUCCESS;
    } else {
      txtLog().debug("SYS CheckGroundStationCommandCondition command not matched: %s != %s",
                     command_str.c_str(), expected_command.c_str());
      return BT::NodeStatus::FAILURE;
    }

  } catch (const std::exception& e) {
    txtLog().error("SYS CheckGroundStationCommandCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior_node