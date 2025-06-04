#pragma once

#include <nlohmann/json.hpp>
#include <chrono>
#include <string>
#include <memory>

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
  nlohmann::json data;
  std::chrono::steady_clock::time_point timestamp;

  BehaviorMessage(BehaviorCommand command,
                  std::string tree_name,
                  nlohmann::json data) :
      command(command), tree_name(tree_name), data(data) {}
};

enum class SystemState {
  INITIALIZING,
  READY,
  RUNNING,
  PAUSED,
  ERROR,
  SHUTTING_DOWN
};

} // namespace behavior_core