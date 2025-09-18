#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"

#include <geometry_msgs/msg/point.hpp>

/**
 * 检查解锁状态条件节点 - 检查飞机是否已解锁
 */
class CheckArmedCondition : public ConditionBase<CheckArmedCondition> {
 public:
  CheckArmedCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckArmedCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("expected_state", true, "期望的解锁状态")
    };
  }

 protected:
  bool checkCondition() override;
};

/**
 * 检查飞行模式条件节点 - 检查当前飞行模式
 */
class CheckFlightModeCondition : public ConditionBase<CheckFlightModeCondition> {
 public:
  CheckFlightModeCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckFlightModeCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("expected_mode", "OFFBOARD", "期望的飞行模式")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  FlightMode stringToFlightMode(const std::string& mode_str);
};

/**
 * 检查GPS状态条件节点 - 检查GPS定位状态
 */
class CheckGPSCondition : public ConditionBase<CheckGPSCondition> {
 public:
  CheckGPSCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckGPSCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("min_fix_type", 3, "最低GPS定位类型 (3:3D_FIX)"),
        BT::InputPort<int>("min_satellites", 6, "最低卫星数量")
    };
  }

 protected:
  bool checkCondition() override;
};

/**
 * 检查电池状态条件节点 - 检查电池电量
 */
class CheckBatteryCondition : public ConditionBase<CheckBatteryCondition> {
 public:
  CheckBatteryCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckBatteryCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("min_percentage", 20.0f, "最低电量百分比"),
        BT::InputPort<float>("min_voltage", 11.0f, "最低电压(V)")
    };
  }

 protected:
  bool checkCondition() override;
};

/**
 * 检查起飞准备条件节点 - 综合检查起飞条件
 */
class CheckTakeoffReadyCondition : public ConditionBase<CheckTakeoffReadyCondition> {
 public:
  CheckTakeoffReadyCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckTakeoffReadyCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("min_battery", 25.0f, "最低电量百分比"),
        BT::InputPort<int>("min_gps_fix", 3, "最低GPS定位类型"),
        BT::InputPort<bool>("check_sensors", true, "检查传感器状态")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool checkSensorHealth();
};

/**
 * 检查降落条件节点 - 检查是否可以安全降落
 */
class CheckLandingCondition : public ConditionBase<CheckLandingCondition> {
 public:
  CheckLandingCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckLandingCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("min_altitude", 2.0f, "最低安全降落高度(米)"),
        BT::InputPort<bool>("check_ground_clear", true, "检查地面是否清楚"),
        BT::InputPort<float>("landing_zone_radius", 10.0f, "降落区域半径(米)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isGroundClear(float radius);
  bool isSafeAltitude(float min_altitude);
};

/**
 * 检查高度条件节点 - 检查飞行高度是否在指定范围内
 */
class CheckAltitudeCondition : public ConditionBase<CheckAltitudeCondition> {
 public:
  CheckAltitudeCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckAltitudeCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("min_altitude", 0.0f, "最低高度(米)"),
        BT::InputPort<float>("max_altitude", 1000.0f, "最高高度(米)"),
        BT::InputPort<bool>("relative_to_home", true, "相对于Home点")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  float getCurrentAltitude(bool relative_to_home);
};

/**
 * 检查飞行速度条件节点 - 检查飞行速度是否在合理范围内
 */
class CheckSpeedCondition : public ConditionBase<CheckSpeedCondition> {
 public:
  CheckSpeedCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckSpeedCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("min_speed", 0.0f, "最低速度(m/s)"),
        BT::InputPort<float>("max_speed", 20.0f, "最高速度(m/s)"),
        BT::InputPort<std::string>("speed_type", "ground", "速度类型 (ground, air, vertical)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  float getCurrentSpeed(const std::string& speed_type);
};

/**
 * 检查Offboard模式条件节点 - 检查是否可以进入Offboard模式
 */
class CheckOffboardReadyCondition : public ConditionBase<CheckOffboardReadyCondition> {
 public:
  CheckOffboardReadyCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckOffboardReadyCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("require_armed", true, "要求已解锁"),
        BT::InputPort<bool>("require_gps", true, "要求GPS定位")
    };
  }

 protected:
  bool checkCondition() override;
};

/**
 * 检查紧急状态条件节点 - 检查是否处于紧急状态
 */
class CheckEmergencyCondition : public ConditionBase<CheckEmergencyCondition> {
 public:
  CheckEmergencyCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckEmergencyCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("critical_battery", 10.0f, "临界电量百分比"),
        BT::InputPort<bool>("check_communication", true, "检查通信状态"),
        BT::InputPort<float>("max_wind_speed", 15.0f, "最大风速(m/s)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool hasCriticalBattery(float threshold);
  bool hasCommunicationLoss();
  bool hasExcessiveWind(float max_wind);
  bool hasSystemFailure();
};

/**
 * 检查返航条件节点 - 检查是否需要返航
 */
class CheckRTLCondition : public ConditionBase<CheckRTLCondition> {
 public:
  CheckRTLCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckRTLCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("battery_threshold", 30.0f, "返航电量阈值"),
        BT::InputPort<float>("distance_threshold", 1000.0f, "返航距离阈值(米)"),
        BT::InputPort<int>("mission_timeout", 1800, "任务超时时间(秒)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isBatteryLow(float threshold);
  bool isTooFarFromHome(float max_distance);
  bool isMissionTimeout(int timeout_seconds);
};

/**
 * 检查飞行环境条件节点 - 检查飞行环境是否适合
 */
class CheckFlightEnvironmentCondition : public ConditionBase<CheckFlightEnvironmentCondition> {
 public:
  CheckFlightEnvironmentCondition(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckFlightEnvironmentCondition>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("max_wind_speed", 12.0f, "最大风速(m/s)"),
        BT::InputPort<float>("min_visibility", 1000.0f, "最小能见度(米)"),
        BT::InputPort<bool>("check_no_fly_zone", true, "检查禁飞区")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isWindAcceptable(float max_wind);
  bool isVisibilityAcceptable(float min_visibility);
  bool isInNoFlyZone();
};