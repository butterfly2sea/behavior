#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/msg/object_location.hpp>
#include <custom_msgs/msg/object_attack_designate.hpp>

#include <future>

/**
 * 跟踪攻击控制动作节点 - 控制跟踪或攻击目标
 */
class TraceAttackControl : public StatefulActionBase<TraceAttackControl> {
 public:
  TraceAttackControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<TraceAttackControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("frame", 1, "坐标系框架"),
        BT::InputPort<int>("command", 1, "控制命令 (1:开始, 2:暂停, 3:停止)"),
        BT::InputPort<int>("mode", 0, "模式 (0:跟踪, 1:攻击)"),
        BT::InputPort<uint8_t>("target_id", 0, "目标ID")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr> service_future_;
  int frame_;
  int command_;
  int mode_;
  uint8_t target_id_;
  bool service_called_ = false;
};

/**
 * 导航控制动作节点 - 控制导航系统
 */
class NavigationControl : public StatefulActionBase<NavigationControl> {
 public:
  NavigationControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<NavigationControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("frame", 1, "坐标系框架"),
        BT::InputPort<int>("command", 1, "控制命令 (1:开始, 2:暂停, 3:停止)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::future<custom_msgs::srv::CommandString::Response::SharedPtr> service_future_;
  int frame_;
  int command_;
  bool service_called_ = false;
};

/**
 * 速度控制动作节点 - 基于速度的飞行控制
 */
class VelocityControl : public StatefulActionBase<VelocityControl> {
 public:
  VelocityControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<VelocityControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Vector3>("velocity", "目标速度向量"),
        BT::InputPort<float>("yaw_rate", 0.0f, "偏航角速度(deg/s)"),
        BT::InputPort<float>("duration", 1.0f, "控制持续时间(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  geometry_msgs::msg::Vector3 target_velocity_;
  float yaw_rate_;
  float duration_;
  std::chrono::steady_clock::time_point control_start_time_;

  void sendVelocityControl();
};

/**
 * 姿态控制动作节点 - 控制飞行器姿态
 */
class AttitudeControl : public StatefulActionBase<AttitudeControl> {
 public:
  AttitudeControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<AttitudeControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("roll", 0.0f, "横滚角(度)"),
        BT::InputPort<float>("pitch", 0.0f, "俯仰角(度)"),
        BT::InputPort<float>("yaw", 0.0f, "偏航角(度)"),
        BT::InputPort<float>("thrust", 0.5f, "推力 (0-1)"),
        BT::InputPort<float>("duration", 1.0f, "控制持续时间(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  float target_roll_;
  float target_pitch_;
  float target_yaw_;
  float target_thrust_;
  float duration_;
  std::chrono::steady_clock::time_point control_start_time_;

  void sendAttitudeControl();
};

/**
 * 手动控制接管动作节点 - 处理手动控制输入
 */
class ManualControlOverride : public StatefulActionBase<ManualControlOverride> {
 public:
  ManualControlOverride(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<ManualControlOverride>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("enable_override", true, "启用手动接管"),
        BT::InputPort<float>("timeout", 30.0f, "接管超时时间(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  bool enable_override_;
  float timeout_;
  std::chrono::steady_clock::time_point override_start_time_;
  std::chrono::steady_clock::time_point last_manual_input_time_;

  bool hasManualInput();
  void processManualInput();
  bool isOverrideActive();
};

/**
 * 避障控制动作节点 - 实时避障控制
 */
class ObstacleAvoidanceControl : public StatefulActionBase<ObstacleAvoidanceControl> {
 public:
  ObstacleAvoidanceControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<ObstacleAvoidanceControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("detection_range", 20.0f, "障碍物检测范围(米)"),
        BT::InputPort<float>("avoidance_margin", 5.0f, "避障安全余量(米)"),
        BT::InputPort<geometry_msgs::msg::Point>("target", "目标点"),
        BT::InputPort<bool>("enable_avoidance", true, "启用避障")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  float detection_range_;
  float avoidance_margin_;
  geometry_msgs::msg::Point target_point_;
  bool enable_avoidance_;

  std::vector<geometry_msgs::msg::Point> detected_obstacles_;
  geometry_msgs::msg::Point avoidance_waypoint_;

  void detectObstacles();
  geometry_msgs::msg::Point calculateAvoidanceWaypoint();
  bool hasObstacleInPath();
  void sendAvoidanceControl();
};

/**
 * 紧急控制动作节点 - 处理紧急情况的控制
 */
class EmergencyControl : public StatefulActionBase<EmergencyControl> {
 public:
  EmergencyControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<EmergencyControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("emergency_type", "STOP", "紧急类型 (STOP, RTL, LAND, HOVER)"),
        BT::InputPort<bool>("immediate", true, "立即执行"),
        BT::InputPort<float>("emergency_altitude", 50.0f, "紧急高度(米)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::string emergency_type_;
  bool immediate_;
  float emergency_altitude_;

  void executeEmergencyStop();
  void executeEmergencyRTL();
  void executeEmergencyLand();
  void executeEmergencyHover();
};

/**
 * 相机控制动作节点 - 控制相机和云台
 */
class CameraControl : public SyncActionBase<CameraControl> {
 public:
  CameraControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<CameraControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("pan", 0.0f, "云台水平角度(度)"),
        BT::InputPort<float>("tilt", -90.0f, "云台垂直角度(度)"),
        BT::InputPort<float>("zoom", 1.0f, "缩放倍数"),
        BT::InputPort<bool>("record", false, "开始录制"),
        BT::InputPort<bool>("take_photo", false, "拍照")
    };
  }

 protected:
  BT::NodeStatus execute() override;

 private:
  void sendCameraCommand(float pan, float tilt, float zoom, bool record, bool photo);
};

/**
 * 图像分发控制动作节点 - 控制图像分发
 */
class ImageDistributeControl : public SyncActionBase<ImageDistributeControl> {
 public:
  ImageDistributeControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<ImageDistributeControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<uint8_t>>("target_vehicles", "目标飞机列表"),
        BT::InputPort<int>("image_type", 1, "图像类型 (1:可见光, 2:红外)"),
        BT::InputPort<float>("distribution_rate", 10.0f, "分发频率(Hz)")
    };
  }

 protected:
  BT::NodeStatus execute() override;

 private:
  void publishImageDistribution(const std::vector<uint8_t>& targets,
                                int image_type,
                                float rate);
};

/**
 * 系统自检控制动作节点 - 执行系统自检
 */
class SystemSelfCheck : public StatefulActionBase<SystemSelfCheck> {
 public:
  SystemSelfCheck(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<SystemSelfCheck>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<bool>("check_sensors", true, "检查传感器"),
        BT::InputPort<bool>("check_actuators", true, "检查执行器"),
        BT::InputPort<bool>("check_communication", true, "检查通信"),
        BT::InputPort<bool>("check_battery", true, "检查电池"),
        BT::OutputPort<bool>("check_result", "自检结果")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  bool check_sensors_;
  bool check_actuators_;
  bool check_communication_;
  bool check_battery_;

  enum class CheckPhase {
    SENSORS,
    ACTUATORS,
    COMMUNICATION,
    BATTERY,
    COMPLETE
  };

  CheckPhase current_phase_;
  bool overall_result_;

  bool checkSensors();
  bool checkActuators();
  bool checkCommunication();
  bool checkBattery();
  void moveToNextPhase();
};