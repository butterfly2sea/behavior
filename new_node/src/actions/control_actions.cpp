#include "behavior_node/actions/control_actions.hpp"
#include <log/Logger.hpp>

namespace behavior_node {

// ================================ LockControlAction ================================

LockControlAction::LockControlAction(const std::string& n, const BT::NodeConfig& config,
                                     std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("CTRL LockControlAction created: %s", n.c_str());
}

BT::PortsList LockControlAction::providedPorts() {
  return {
      BT::InputPort<bool>("state", true, "Lock state: true=armed, false=disarmed")
  };
}

BT::NodeStatus LockControlAction::tick() {
  try {
    bool lock_state;
    if (!getInput("state", lock_state)) {
      txtLog().error("CTRL LockControlAction failed to get lock state");
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("CTRL LockControlAction setting lock state: %s",
                  lock_state ? "ARMED" : "DISARMED");

    auto future = deps_->ros_comm->callLockControl(lock_state);

    // 等待服务响应
    if (rclcpp::spin_until_future_complete(deps_->ros_comm->getNode(), future,
                                           std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
      txtLog().error("CTRL LockControlAction service call timeout");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response || !response->success) {
      txtLog().error("CTRL LockControlAction service call failed");
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("CTRL LockControlAction completed successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("CTRL LockControlAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ FlightModeControlAction ================================

FlightModeControlAction::FlightModeControlAction(const std::string& n, const BT::NodeConfig& config,
                                                 std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("CTRL FlightModeControlAction created: %s", n.c_str());
}

BT::PortsList FlightModeControlAction::providedPorts() {
  return {
      BT::InputPort<std::string>("mode", "OFFBOARD", "Flight mode (OFFBOARD, MANUAL, AUTO, etc.)"),
      BT::InputPort<float>("param7", 0.0f, "Additional parameter")
  };
}

BT::NodeStatus FlightModeControlAction::tick() {
  try {
    std::string mode_str;
    float param7 = 0.0f;

    if (!getInput("mode", mode_str)) {
      txtLog().error("CTRL FlightModeControlAction failed to get mode");
      return BT::NodeStatus::FAILURE;
    }
    getInput("param7", param7);

    // 将字符串模式转换为数值
    uint8_t mode_val = stringToFlightMode(mode_str);
    if (mode_val == 255) {
      txtLog().error("CTRL FlightModeControlAction invalid flight mode: %s", mode_str.c_str());
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("CTRL FlightModeControlAction setting mode: %s (%d)", mode_str.c_str(), mode_val);

    auto future = deps_->ros_comm->callFlightModeControl(mode_val, param7);

    // 等待服务响应
    if (rclcpp::spin_until_future_complete(deps_->ros_comm->getNode(), future,
                                           std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
      txtLog().error("CTRL FlightModeControlAction service call timeout");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response || !response->success) {
      txtLog().error("CTRL FlightModeControlAction service call failed");
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("CTRL FlightModeControlAction completed successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("CTRL FlightModeControlAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

uint8_t FlightModeControlAction::stringToFlightMode(const std::string& mode) const {
  static const std::map<std::string, uint8_t> mode_map = {
      {"MANUAL", 0},
      {"ALTCTL", 1},
      {"POSCTL", 2},
      {"AUTO", 3},
      {"ACRO", 4},
      {"OFFBOARD", 5},
      {"STABILIZED", 6},
      {"RATTITUDE", 7},
      {"LAND", 8},
      {"RTL", 9},
      {"MISSION", 10},
      {"LOITER", 11}
  };

  auto it = mode_map.find(mode);
  return (it != mode_map.end()) ? it->second : 255;
}

// ================================ TakeoffControlAction ================================

TakeoffControlAction::TakeoffControlAction(const std::string& n, const BT::NodeConfig& config,
                                           std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("CTRL TakeoffControlAction created: %s", n.c_str());
}

BT::PortsList TakeoffControlAction::providedPorts() {
  return {
      BT::InputPort<float>("altitude", 10.0f, "Takeoff altitude in meters")
  };
}

BT::NodeStatus TakeoffControlAction::tick() {
  try {
    float altitude;
    if (!getInput("altitude", altitude)) {
      txtLog().error("CTRL TakeoffControlAction failed to get altitude");
      return BT::NodeStatus::FAILURE;
    }

    if (altitude <= 0.0f || altitude > 1000.0f) {
      txtLog().error("CTRL TakeoffControlAction invalid altitude: %.2f", altitude);
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("CTRL TakeoffControlAction taking off to altitude: %.2f m", altitude);

    auto future = deps_->ros_comm->callTakeoffControl(altitude);

    // 等待服务响应
    if (rclcpp::spin_until_future_complete(deps_->ros_comm->getNode(), future,
                                           std::chrono::seconds(30)) != rclcpp::FutureReturnCode::SUCCESS) {
      txtLog().error("CTRL TakeoffControlAction service call timeout");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response || !response->success) {
      txtLog().error("CTRL TakeoffControlAction service call failed");
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("CTRL TakeoffControlAction takeoff command sent successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("CTRL TakeoffControlAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ LandControlAction ================================

LandControlAction::LandControlAction(const std::string& n, const BT::NodeConfig& config,
                                     std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("CTRL LandControlAction created: %s", n.c_str());
}

BT::PortsList LandControlAction::providedPorts() {
  return {};  // 降落不需要参数
}

BT::NodeStatus LandControlAction::tick() {
  try {
    txtLog().info("CTRL LandControlAction initiating landing");

    auto future = deps_->ros_comm->callLandControl();

    // 等待服务响应
    if (rclcpp::spin_until_future_complete(deps_->ros_comm->getNode(), future,
                                           std::chrono::seconds(10)) != rclcpp::FutureReturnCode::SUCCESS) {
      txtLog().error("CTRL LandControlAction service call timeout");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response || !response->success) {
      txtLog().error("CTRL LandControlAction service call failed");
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("CTRL LandControlAction landing command sent successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("CTRL LandControlAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ================================ TraceAttackControlAction ================================

TraceAttackControlAction::TraceAttackControlAction(const std::string& n, const BT::NodeConfig& config,
                                                   std::shared_ptr<NodeDependencies> deps)
    : BaseActionNode(n, config, deps) {
  txtLog().debug("CTRL TraceAttackControlAction created: %s", n.c_str());
}

BT::PortsList TraceAttackControlAction::providedPorts() {
  return {
      BT::InputPort<uint8_t>("frame", 1, "Frame type"),
      BT::InputPort<uint8_t>("command", 1, "Command type"),
      BT::InputPort<uint8_t>("mode", 0, "Attack mode"),
      BT::InputPort<int32_t>("target_id", -1, "Target ID")
  };
}

BT::NodeStatus TraceAttackControlAction::tick() {
  try {
    uint8_t frame = 1;
    uint8_t command = 1;
    uint8_t mode = 0;
    int32_t target_id = -1;

    getInput("frame", frame);
    getInput("command", command);
    getInput("mode", mode);
    getInput("target_id", target_id);

    txtLog().info("CTRL TraceAttackControlAction frame=%d, command=%d, mode=%d, target_id=%d",
                  frame, command, mode, target_id);

    auto future = deps_->ros_comm->callTraceAttackControl(frame, command);

    // 等待服务响应
    if (rclcpp::spin_until_future_complete(deps_->ros_comm->getNode(), future,
                                           std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS) {
      txtLog().error("CTRL TraceAttackControlAction service call timeout");
      return BT::NodeStatus::FAILURE;
    }

    auto response = future.get();
    if (!response || !response->success) {
      txtLog().error("CTRL TraceAttackControlAction service call failed");
      return BT::NodeStatus::FAILURE;
    }

    txtLog().info("CTRL TraceAttackControlAction completed successfully");
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    txtLog().error("CTRL TraceAttackControlAction exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior_node