#include "behavior_node/conditions/flight_conditions.hpp"
#include <log/Logger.hpp>
#include <cmath>

namespace behavior_node {

// ================================ CheckFlightReadyCondition ================================

CheckFlightReadyCondition::CheckFlightReadyCondition(const std::string& n, const BT::NodeConfig& config,
                                                     std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("FLIGHT CheckFlightReadyCondition created: %s", n.c_str());
}

BT::PortsList CheckFlightReadyCondition::providedPorts() {
  return {
      BT::InputPort<float>("min_battery_level", 20.0f, "Minimum battery level percentage"),
      BT::InputPort<bool>("check_gps", true, "Check GPS fix status"),
      BT::InputPort<bool>("check_compass", true, "Check compass calibration"),
      BT::InputPort<bool>("check_sensors", true, "Check sensor health")
  };
}

BT::NodeStatus CheckFlightReadyCondition::tick() {
  try {
    float min_battery = 20.0f;
    bool check_gps = true;
    bool check_compass = true;
    bool check_sensors = true;

    getInput("min_battery_level", min_battery);
    getInput("check_gps", check_gps);
    getInput("check_compass", check_compass);
    getInput("check_sensors", check_sensors);

    // 检查电池电量
    if (auto battery_state = deps_->cache->getBatteryState()) {
      if (battery_state->percentage < min_battery) {
        txtLog().warning("FLIGHT CheckFlightReadyCondition battery level too low: %.1f%% < %.1f%%",
                         battery_state->percentage, min_battery);
        return BT::NodeStatus::FAILURE;
      }
    } else {
      txtLog().warning("FLIGHT CheckFlightReadyCondition no battery state available");
      return BT::NodeStatus::FAILURE;
    }

    // 检查GPS状态
    if (check_gps) {
      if (auto gps_status = deps_->cache->getGPSStatus()) {
        if (gps_status->fix_type < 3) {  // 需要3D GPS fix
          txtLog().warning("FLIGHT CheckFlightReadyCondition GPS fix insufficient: %d", gps_status->fix_type);
          return BT::NodeStatus::FAILURE;
        }
      } else {
        txtLog().warning("FLIGHT CheckFlightReadyCondition no GPS status available");
        return BT::NodeStatus::FAILURE;
      }
    }

    // 检查罗盘校准状态
    if (check_compass) {
      if (auto compass_status = deps_->cache->getCompassStatus()) {
        if (!compass_status->calibrated) {
          txtLog().warning("FLIGHT CheckFlightReadyCondition compass not calibrated");
          return BT::NodeStatus::FAILURE;
        }
      } else {
        txtLog().warning("FLIGHT CheckFlightReadyCondition no compass status available");
        return BT::NodeStatus::FAILURE;
      }
    }

    // 检查传感器健康状态
    if (check_sensors) {
      if (auto health_status = deps_->cache->getHealthStatus()) {
        if (!health_status->sensors_healthy) {
          txtLog().warning("FLIGHT CheckFlightReadyCondition sensors not healthy");
          return BT::NodeStatus::FAILURE;
        }
      } else {
        txtLog().warning("FLIGHT CheckFlightReadyCondition no health status available");
        return BT::NodeStatus::FAILURE;
      }
    }

    txtLog().debug("FLIGHT CheckFlightReadyCondition all checks passed");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FLIGHT CheckFlightReadyCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckAltitudeCondition ================================

CheckAltitudeCondition::CheckAltitudeCondition(const std::string& n, const BT::NodeConfig& config,
                                               std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("FLIGHT CheckAltitudeCondition created: %s", n.c_str());
}

BT::PortsList CheckAltitudeCondition::providedPorts() {
  return {
      BT::InputPort<float>("min_altitude", 5.0f, "Minimum altitude in meters"),
      BT::InputPort<float>("max_altitude", 500.0f, "Maximum altitude in meters"),
      BT::InputPort<float>("tolerance", 2.0f, "Altitude tolerance in meters")
  };
}

BT::NodeStatus CheckAltitudeCondition::tick() {
  try {
    float min_alt = 5.0f;
    float max_alt = 500.0f;
    float tolerance = 2.0f;

    getInput("min_altitude", min_alt);
    getInput("max_altitude", max_alt);
    getInput("tolerance", tolerance);

    auto current_pos = deps_->cache->getCurrentPosition();
    if (!current_pos) {
      txtLog().warning("FLIGHT CheckAltitudeCondition no position available");
      return BT::NodeStatus::FAILURE;
    }

    float current_alt = current_pos->z;

    // 检查高度是否在允许范围内
    if (current_alt < (min_alt - tolerance)) {
      txtLog().warning("FLIGHT CheckAltitudeCondition altitude too low: %.2f < %.2f",
                       current_alt, min_alt);
      return BT::NodeStatus::FAILURE;
    }

    if (current_alt > (max_alt + tolerance)) {
      txtLog().warning("FLIGHT CheckAltitudeCondition altitude too high: %.2f > %.2f",
                       current_alt, max_alt);
      return BT::NodeStatus::FAILURE;
    }

    txtLog().debug("FLIGHT CheckAltitudeCondition altitude OK: %.2f m", current_alt);
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FLIGHT CheckAltitudeCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckFlightModeCondition ================================

CheckFlightModeCondition::CheckFlightModeCondition(const std::string& n, const BT::NodeConfig& config,
                                                   std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("FLIGHT CheckFlightModeCondition created: %s", n.c_str());
}

BT::PortsList CheckFlightModeCondition::providedPorts() {
  return {
      BT::InputPort<std::string>("required_mode", "OFFBOARD", "Required flight mode")
  };
}

BT::NodeStatus CheckFlightModeCondition::tick() {
  try {
    std::string required_mode;
    if (!getInput("required_mode", required_mode)) {
      txtLog().error("FLIGHT CheckFlightModeCondition failed to get required_mode");
      return BT::NodeStatus::FAILURE;
    }

    auto flight_state = deps_->cache->getFlightState();
    if (!flight_state) {
      txtLog().warning("FLIGHT CheckFlightModeCondition no flight state available");
      return BT::NodeStatus::FAILURE;
    }

    // 将模式代码转换为字符串进行比较
    std::string current_mode = flightModeToString(flight_state->mode);

    if (current_mode != required_mode) {
      txtLog().warning("FLIGHT CheckFlightModeCondition mode mismatch: %s != %s",
                       current_mode.c_str(), required_mode.c_str());
      return BT::NodeStatus::FAILURE;
    }

    txtLog().debug("FLIGHT CheckFlightModeCondition mode OK: %s", current_mode.c_str());
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FLIGHT CheckFlightModeCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

std::string CheckFlightModeCondition::flightModeToString(uint8_t mode) const {
  static const std::map<uint8_t, std::string> mode_map = {
      {0, "MANUAL"},
      {1, "ALTCTL"},
      {2, "POSCTL"},
      {3, "AUTO"},
      {4, "ACRO"},
      {5, "OFFBOARD"},
      {6, "STABILIZED"},
      {7, "RATTITUDE"},
      {8, "LAND"},
      {9, "RTL"},
      {10, "MISSION"},
      {11, "LOITER"}
  };

  auto it = mode_map.find(mode);
  return (it != mode_map.end()) ? it->second : "UNKNOWN";
}

// ================================ CheckArmedCondition ================================

CheckArmedCondition::CheckArmedCondition(const std::string& n, const BT::NodeConfig& config,
                                         std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("FLIGHT CheckArmedCondition created: %s", n.c_str());
}

BT::PortsList CheckArmedCondition::providedPorts() {
  return {
      BT::InputPort<bool>("should_be_armed", true, "Expected armed state")
  };
}

BT::NodeStatus CheckArmedCondition::tick() {
  try {
    bool should_be_armed = true;
    getInput("should_be_armed", should_be_armed);

    auto flight_state = deps_->cache->getFlightState();
    if (!flight_state) {
      txtLog().warning("FLIGHT CheckArmedCondition no flight state available");
      return BT::NodeStatus::FAILURE;
    }

    bool is_armed = flight_state->armed;

    if (is_armed != should_be_armed) {
      txtLog().warning("FLIGHT CheckArmedCondition armed state mismatch: %s != %s",
                       is_armed ? "ARMED" : "DISARMED",
                       should_be_armed ? "ARMED" : "DISARMED");
      return BT::NodeStatus::FAILURE;
    }

    txtLog().debug("FLIGHT CheckArmedCondition armed state OK: %s",
                   is_armed ? "ARMED" : "DISARMED");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FLIGHT CheckArmedCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ CheckLandedCondition ================================

CheckLandedCondition::CheckLandedCondition(const std::string& n, const BT::NodeConfig& config,
                                           std::shared_ptr<NodeDependencies> deps)
    : BaseConditionNode(n, config, deps) {
  txtLog().debug("FLIGHT CheckLandedCondition created: %s", n.c_str());
}

BT::PortsList CheckLandedCondition::providedPorts() {
  return {
      BT::InputPort<float>("ground_altitude_threshold", 2.0f, "Altitude threshold for ground detection")
  };
}

BT::NodeStatus CheckLandedCondition::tick() {
  try {
    float ground_threshold = 2.0f;
    getInput("ground_altitude_threshold", ground_threshold);

    auto flight_state = deps_->cache->getFlightState();
    auto position = deps_->cache->getCurrentPosition();

    if (!flight_state || !position) {
      txtLog().warning("FLIGHT CheckLandedCondition missing flight state or position");
      return BT::NodeStatus::FAILURE;
    }

    // 检查是否解锁（landed状态下应该是解锁的）
    bool is_disarmed = !flight_state->armed;

    // 检查高度是否接近地面
    bool is_low_altitude = position->z <= ground_threshold;

    // 检查是否处于着陆模式或已着陆
    bool in_land_mode = (flight_state->mode == 8);  // LAND mode

    bool is_landed = is_disarmed && is_low_altitude;

    if (!is_landed && !in_land_mode) {
      txtLog().debug("FLIGHT CheckLandedCondition not landed: armed=%s, alt=%.2f, mode=%d",
                     flight_state->armed ? "true" : "false", position->z, flight_state->mode);
      return BT::NodeStatus::FAILURE;
    }

    txtLog().debug("FLIGHT CheckLandedCondition landed confirmed: armed=%s, alt=%.2f",
                   flight_state->armed ? "true" : "false", position->z);
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("FLIGHT CheckLandedCondition exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior_node