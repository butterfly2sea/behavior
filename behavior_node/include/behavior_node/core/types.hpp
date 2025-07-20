#pragma once

#include <nlohmann/json.hpp>
#include <chrono>
#include <string>

namespace behavior_core {
enum class BehaviorCommand {
  NONE,
  LOAD_TREE,
  STOP_TREE,
  PAUSE_TREE,
  RESUME_TREE,
  SHUTDOWN
};

struct BehaviorMessage {
  BehaviorCommand command;
  std::string tree_name;
  std::unordered_map<std::string, nlohmann::json> data;
  std::chrono::steady_clock::time_point timestamp;

  BehaviorMessage(BehaviorCommand command, std::string tree_name, std::unordered_map<std::string, nlohmann::json> data)
      : command(command), tree_name(std::move(tree_name)), data(std::move(data)) {}
};

enum class SystemState {
  INITIALIZING,
  READY,
  RUNNING,
  PAUSED,
  ERROR,
  SHUTTING_DOWN
};

// JSON任务参数结构
struct TaskAction {
  int groupid;
  int id;
  std::string name;
  std::unordered_map<std::string, nlohmann::json> params;
  std::unordered_map<std::string, nlohmann::json> triggers;
};

struct TaskStage {
  std::string name;
  int sn;
  std::string cmd;  // start, pause, continue, stop
  std::vector<TaskAction> actions;
};

struct TaskMission {
  std::vector<TaskStage> stages;
};

} // namespace behavior_core