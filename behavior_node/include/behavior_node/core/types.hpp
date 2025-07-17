#pragma once

#include <nlohmann/json.hpp>
#include <custom_msgs/msg/command_request.hpp>
#include <chrono>
#include <string>
#include <unordered_map>

namespace behavior_core {

enum class SystemState {
  INITIALIZING,
  RUNNING,
  PAUSED,
  ERROR,
  SHUTTING_DOWN
};

enum class TaskStatus {
  NO_START = 0,
  ONGOING = 1,
  FAILED = 2,
  COMPLETE = 3,
  NOT_READY = 4
};

enum class MessageType {
  TREE_EXECUTION,
  COMMAND,
  ATTACK_DESIGNATE,
  JOYSTICK_CONTROL
};

enum class CommandType {
  TAKEOFF = 1,
  LAND = 2,
  RTL = 3,
  LOITER = 4,
  ARM = 5,
  DISARM = 6,
  SET_MODE = 7,
  STOP_TREE = 99
};

// 基础消息类型
struct BaseMessage {
  MessageType type;
  std::chrono::steady_clock::time_point timestamp;

  BaseMessage(MessageType t) : type(t), timestamp(std::chrono::steady_clock::now()) {}
  virtual ~BaseMessage() = default;
};

// 行为树消息
struct TreeMessage : public BaseMessage {
  std::string tree_name;
  std::string action_name;
  int group_id{1};
  int vehicle_id{0};
  std::unordered_map<std::string, nlohmann::json> parameters;

  TreeMessage() : BaseMessage(MessageType::TREE_EXECUTION) {}
};

// 控制消息
struct ControlMessage : public BaseMessage {
  CommandType command_type;
  int vehicle_id{0};
  custom_msgs::msg::CommandRequest data;

  ControlMessage() : BaseMessage(MessageType::COMMAND) {}
};

// 攻击消息
struct AttackMessage : public BaseMessage {
  int target_id{0};
  int attack_type{0};

  AttackMessage() : BaseMessage(MessageType::ATTACK_DESIGNATE) {}
};

// 摇杆控制消息
struct JoystickMessage : public BaseMessage {
  // 摇杆数据将在data_cache中管理
  JoystickMessage() : BaseMessage(MessageType::JOYSTICK_CONTROL) {}
};

} // namespace behavior_core