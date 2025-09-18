#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>

namespace behavior_core {

// 系统状态枚举
enum class SystemState {
  INITIALIZING,
  IDLE,
  EXECUTING,
  PAUSED,
  ERROR,
  SHUTTING_DOWN
};

// 树执行状态枚举  
enum class TreeState {
  IDLE,
  LOADING,
  RUNNING,
  PAUSED,
  STOPPING,
  FAILED,
  SUCCESS
};

// 行为命令枚举
enum class BehaviorCommand {
  NONE,
  LOAD_TREE,
  START_TREE,
  PAUSE_TREE,
  RESUME_TREE,
  STOP_TREE,
  SET_PARAMETER,
  EMERGENCY_STOP
};

// 任务状态枚举
enum class TaskStatus {
  PENDING,
  ACTIVE,
  PAUSED,
  COMPLETED,
  FAILED,
  CANCELLED
};

// 行为消息结构
struct BehaviorMessage {
  BehaviorCommand command;
  std::string tree_name;
  std::unordered_map<std::string, nlohmann::json> parameters;
  std::chrono::steady_clock::time_point timestamp;

  BehaviorMessage() = default;

  BehaviorMessage(BehaviorCommand cmd,
                  const std::string& name,
                  const std::unordered_map<std::string, nlohmann::json>& params = {})
      : command(cmd), tree_name(name), parameters(params),
        timestamp(std::chrono::steady_clock::now()) {}
};

// 执行上下文结构
struct ExecutionContext {
  TreeState state = TreeState::IDLE;
  std::string tree_name;
  std::string task_name;
  std::chrono::steady_clock::time_point start_time;
  std::chrono::steady_clock::time_point last_tick;
  uint64_t tick_count = 0;
  std::string last_error;
};

// 系统配置结构
struct SystemConfig {
  std::string tree_directory = "tree";
  std::string main_tree_file = "main_behavior_tree.xml";
  double tick_rate = 10.0; // Hz
  uint32_t max_tick_count = 1000000;
  std::chrono::seconds timeout_duration{30};
  bool enable_groot_monitoring = false;
  uint16_t groot_port = 1666;
};

// 飞行状态枚举
enum class FlightState {
  UNKNOWN,
  DISARMED,
  ARMED,
  TAKING_OFF,
  FLYING,
  LANDING,
  LANDED,
  ERROR_STATE
};

// 任务阶段枚举  
enum class MissionPhase {
  NONE,
  PRE_FLIGHT,
  TAKEOFF,
  NAVIGATION,
  SEARCH,
  ATTACK,
  RETURN,
  LANDING
};

// 控制模式枚举
enum class ControlMode {
  MANUAL,
  ALTITUDE_HOLD,
  POSITION_HOLD,
  TAKEOFF,
  LAND,
  LOITER,
  MISSION,
  RTL,
  OFFBOARD,
  STABILIZE
};

} // namespace behavior_core