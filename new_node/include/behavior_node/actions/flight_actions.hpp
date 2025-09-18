#pragma once

#include "behavior_node/base/base_nodes.hpp"
#include "behavior_node/data/base_enum.hpp"

#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/srv/command_bool.hpp>
#include <custom_msgs/srv/command_long.hpp>

#include <future>

/**
 * 锁定控制动作节点 - 解锁/上锁飞机
 */
class LockControl : public SyncActionBase<LockControl> {
 public:
  LockControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<LockControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("state", 1, "锁定状态 (0:上锁, 1:解锁)")
    };
  }

 protected:
  BT::NodeStatus execute() override;
};

/**
 * 飞行模式控制动作节点 - 设置飞行模式
 */
class FlightModeControl : public StatefulActionBase<FlightModeControl> {
 public:
  FlightModeControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<FlightModeControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("mode", "OFFBOARD", "飞行模式 (MANUAL, ALTITUDE_HOLD, POSITION_HOLD, TAKEOFF, LAND, LOITER, MISSION, RTL, OFFBOARD, STABILIZE)"),
        BT::InputPort<float>("param7", 0.0f, "额外参数(起飞高度等)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr> service_future_;
  FlightMode target_mode_;
  bool waiting_for_response_ = false;
};

/**
 * 起飞动作节点 - 执行起飞操作
 */
class TakeOffAction : public StatefulActionBase<TakeOffAction> {
 public:
  TakeOffAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<TakeOffAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("alt", 10.0f, "起飞高度(米)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr> service_future_;
  float target_altitude_;
  bool service_called_ = false;
  std::chrono::steady_clock::time_point takeoff_start_time_;
};

/**
 * 降落动作节点 - 执行降落操作
 */
class LandAction : public StatefulActionBase<LandAction> {
 public:
  LandAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<LandAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("descent_rate", 1.0f, "下降速度(m/s)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr> service_future_;
  float descent_rate_;
  bool service_called_ = false;
  std::chrono::steady_clock::time_point land_start_time_;
};

/**
 * 悬停动作节点 - 在当前位置悬停指定时间
 */
class LoiterAction : public StatefulActionBase<LoiterAction> {
 public:
  LoiterAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<LoiterAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("duration", 5.0f, "悬停时长(秒)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  float duration_;
  geometry_msgs::msg::Point hover_position_;
  std::chrono::steady_clock::time_point loiter_start_time_;
};

/**
 * Offboard控制动作节点 - 发送位置控制指令
 */
class OffboardControl : public SyncActionBase<OffboardControl> {
 public:
  OffboardControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<OffboardControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("position", "目标位置"),
        BT::InputPort<float>("yaw", 0.0f, "偏航角(度)"),
        BT::InputPort<int>("frame", 1, "坐标系 (1:LOCAL_NED)"),
        BT::InputPort<int>("type_mask", 0, "控制掩码")
    };
  }

 protected:
  BT::NodeStatus execute() override;

 private:
  custom_msgs::msg::OffboardCtrl createOffboardMessage(
      const geometry_msgs::msg::Point& position,
      float yaw, int frame, int type_mask);
};

/**
 * 紧急停止动作节点 - 执行紧急停止
 */
class EmergencyStopAction : public SyncActionBase<EmergencyStopAction> {
 public:
  EmergencyStopAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<EmergencyStopAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {};
  }

 protected:
  BT::NodeStatus execute() override;
};

/**
 * 返航动作节点 - 返回home点
 */
class RTLAction : public StatefulActionBase<RTLAction> {
 public:
  RTLAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : StatefulActionBase<RTLAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<float>("rtl_altitude", 50.0f, "返航高度(米)")
    };
  }

 protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

 private:
  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr> service_future_;
  float rtl_altitude_;
  bool service_called_ = false;
  geometry_msgs::msg::Point home_position_;
};

/**
 * 设置Home点动作节点 - 设置返航点
 */
class SetHomeAction : public SyncActionBase<SetHomeAction> {
 public:
  SetHomeAction(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<SetHomeAction>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Point>("home_point", "Home点位置"),
        BT::InputPort<bool>("use_current", false, "使用当前位置作为Home点")
    };
  }

 protected:
  BT::NodeStatus execute() override;
};

/**
 * 武器控制动作节点 - 控制武器投放
 */
class WeaponControl : public SyncActionBase<WeaponControl> {
 public:
  WeaponControl(const std::string &name, const BT::NodeConfiguration &config, NodeDependencies deps)
      : SyncActionBase<WeaponControl>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("action", 0, "武器动作 (0:取消, 1:投放)")
    };
  }

 protected:
  BT::NodeStatus execute() override;
};