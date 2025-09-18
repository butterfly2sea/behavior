#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"
#include "behavior_node/core/types.hpp"

#include <custom_msgs/msg/command_request.hpp>

/**
 * 检查任务开始条件节点 - 检查任务是否开始执行
 */
class CheckStartTask : public ConditionBase<CheckStartTask> {
 public:
  CheckStartTask(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckStartTask>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("trigger_type", "command", "触发类型 (command, time, condition)"),
        BT::InputPort<std::string>("trigger_value", "", "触发值"),
        BT::InputPort<int>("timeout", 60, "等待超时(秒)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool checkCommandTrigger(const std::string& value);
  bool checkTimeTrigger(const std::string& value);
  bool checkConditionTrigger(const std::string& value);
  bool isTimeout(int timeout_seconds);
};

/**
 * 检查系统状态条件节点 - 检查系统整体状态
 */
class CheckSystemState : public ConditionBase<CheckSystemState> {
 public:
  CheckSystemState(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckSystemState>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("expected_state", "EXECUTING", "期望系统状态"),
        BT::InputPort<bool>("check_subsystems", true, "检查子系统状态")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  behavior_core::SystemState stringToSystemState(const std::string& state_str);
  bool checkSubsystemsHealth();
};

/**
 * 检查通信状态条件节点 - 检查通信连接状态
 */
class CheckCommunicationStatus : public ConditionBase<CheckCommunicationStatus> {
 public:
  CheckCommunicationStatus(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckCommunicationStatus>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("check_ground_link", true, "检查地面链路"),
        BT::InputPort<bool>("check_inter_vehicle", false, "检查机间通信"),
        BT::InputPort<float>("min_signal_strength", -80.0f, "最小信号强度(dBm)"),
        BT::InputPort<float>("max_latency", 1000.0f, "最大延迟(ms)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isGroundLinkHealthy(float min_signal, float max_latency);
  bool isInterVehicleCommHealthy();
  float getSignalStrength();
  float getCommunicationLatency();
};

/**
 * 检查服务状态条件节点 - 检查ROS服务是否可用
 */
class CheckServiceAvailable : public ConditionBase<CheckServiceAvailable> {
 public:
  CheckServiceAvailable(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckServiceAvailable>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("service_name", "", "服务名称"),
        BT::InputPort<float>("timeout", 2.0f, "检查超时(秒)")
    };
  }

 protected:
  bool checkCondition() override;
};

/**
 * 检查任务超时条件节点 - 检查任务是否超时
 */
class CheckMissionTimeout : public ConditionBase<CheckMissionTimeout> {
 public:
  CheckMissionTimeout(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckMissionTimeout>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("timeout_seconds", 1800, "任务超时时间(秒)"),
        BT::InputPort<bool>("reset_on_new_command", true, "新命令时重置计时")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  std::chrono::steady_clock::time_point mission_start_time_;
  bool hasReceivedNewCommand();
};

/**
 * 检查资源可用性条件节点 - 检查系统资源是否充足
 */
class CheckResourceAvailability : public ConditionBase<CheckResourceAvailability> {
 public:
  CheckResourceAvailability(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckResourceAvailability>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("min_cpu_free", 20.0f, "最小剩余CPU(%)"),
        BT::InputPort<float>("min_memory_free", 100.0f, "最小剩余内存(MB)"),
        BT::InputPort<float>("min_disk_free", 1000.0f, "最小剩余磁盘(MB)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  float getCPUUsage();
  float getMemoryUsage();
  float getDiskUsage();
};

/**
 * 检查地面站指令条件节点 - 检查是否收到地面站指令
 */
class CheckGroundStationCommand : public ConditionBase<CheckGroundStationCommand> {
 public:
  CheckGroundStationCommand(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckGroundStationCommand>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("expected_command", "", "期望的指令类型"),
        BT::InputPort<float>("command_timeout", 10.0f, "指令超时时间(秒)"),
        BT::InputPort<bool>("consume_command", true, "消费指令(检查后清除)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool hasMatchingCommand(const std::string& expected_cmd);
  bool isCommandRecent(float timeout_seconds);
};

/**
 * 检查编队状态条件节点 - 检查编队相关状态
 */
class CheckFormationStatus : public ConditionBase<CheckFormationStatus> {
 public:
  CheckFormationStatus(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckFormationStatus>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("check_formation_intact", true, "检查编队完整性"),
        BT::InputPort<bool>("check_leader_status", true, "检查长机状态"),
        BT::InputPort<float>("max_formation_error", 10.0f, "最大编队误差(米)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isFormationIntact(float max_error);
  bool isLeaderHealthy();
  float calculateFormationError();
};

/**
 * 检查安全区域条件节点 - 检查是否在安全区域内
 */
class CheckSafetyZone : public ConditionBase<CheckSafetyZone> {
 public:
  CheckSafetyZone(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckSafetyZone>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Polygon>("safe_zone", "安全区域边界"),
        BT::InputPort<float>("safety_margin", 50.0f, "安全余量(米)"),
        BT::InputPort<bool>("check_no_fly_zones", true, "检查禁飞区")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isInSafeZone(const geometry_msgs::msg::Polygon& zone, float margin);
  bool isNotInNoFlyZone();
};

/**
 * 检查任务阶段条件节点 - 检查当前任务阶段
 */
class CheckMissionPhase : public ConditionBase<CheckMissionPhase> {
 public:
  CheckMissionPhase(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckMissionPhase>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("expected_phase", "NAVIGATION", "期望的任务阶段"),
        BT::InputPort<bool>("allow_transition", true, "允许阶段转换")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  behavior_core::MissionPhase stringToMissionPhase(const std::string& phase_str);
};

/**
 * 检查错误状态条件节点 - 检查系统是否有错误
 */
class CheckErrorStatus : public ConditionBase<CheckErrorStatus> {
 public:
  CheckErrorStatus(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckErrorStatus>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("error_category", "any", "错误类别 (any, flight, navigation, communication, sensor)"),
        BT::InputPort<int>("max_error_count", 0, "最大允许错误数"),
        BT::InputPort<bool>("include_warnings", false, "包括警告")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  int getErrorCount(const std::string& category, bool include_warnings);
  bool hasCriticalError();
};

/**
 * 检查数据有效性条件节点 - 检查关键数据是否有效
 */
class CheckDataValidity : public ConditionBase<CheckDataValidity> {
 public:
  CheckDataValidity(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckDataValidity>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("data_type", "position", "数据类型 (position, attitude, velocity, sensor)"),
        BT::InputPort<float>("max_age", 1.0f, "最大数据年龄(秒)"),
        BT::InputPort<bool>("check_quality", true, "检查数据质量")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool isDataRecent(const std::string& data_type, float max_age);
  bool isDataQualityGood(const std::string& data_type);
};

/**
 * 检查手动控制条件节点 - 检查是否有手动控制输入
 */
class CheckManualControl : public ConditionBase<CheckManualControl> {
 public:
  CheckManualControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckManualControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("joystick_active", false, "遥控器是否激活"),
        BT::InputPort<float>("input_threshold", 0.1f, "输入阈值"),
        BT::InputPort<float>("timeout", 5.0f, "手动控制超时(秒)")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool hasActiveJoystickInput(float threshold);
  bool isManualControlRecent(float timeout_seconds);
};

/**
 * 检查任务完成条件节点 - 检查任务是否完成
 */
class CheckMissionComplete : public ConditionBase<CheckMissionComplete> {
 public:
  CheckMissionComplete(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : ConditionBase<CheckMissionComplete>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<std::string>>("completion_criteria", "完成条件列表"),
        BT::InputPort<bool>("require_all", true, "要求满足所有条件")
    };
  }

 protected:
  bool checkCondition() override;

 private:
  bool checkCompletionCriterion(const std::string& criterion);
  bool allCriteriaMet(const std::vector<std::string>& criteria);
  bool anyCriteriaMet(const std::vector<std::string>& criteria);
};