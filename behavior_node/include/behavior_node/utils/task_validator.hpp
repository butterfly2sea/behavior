#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <iterator>
#include <numeric>
#include <vector>
#include <unordered_set>
#include <log/Logger.hpp>

class TaskValidator {
 private:
  static std::unordered_set<std::string> valid_commands_;
  static std::unordered_set<std::string> valid_actions_;
  static std::unordered_set<std::string> required_params_;

 public:
  TaskValidator() {
    valid_commands_ = {"start", "set", "stop", "del"};
    valid_actions_ = {
        // 基础飞行
        "TakeOff", "Land", "Hover", "Emergency",
        // 导航任务
        "GoToDst", "WaypointMission", "Navline", "PatternSearch", "FormFly",
        // 特殊任务
        "AutoTrace", "LockCtrl", "SetLine"
    };
    required_params_ = {"vehiType", "spd", "arvDis", "pointTag"};
  }

  // 主要验证接口
  static bool validateTaskFormat(const nlohmann::json &task);
  static bool validateStage(const nlohmann::json &stage);
  static bool validateAction(const nlohmann::json &action);
  static bool validateParameters(const nlohmann::json &params);
  static bool validateTriggers(const nlohmann::json &triggers);

  // 辅助验证方法
  static bool isValidCommand(const std::string &cmd);
  static bool isValidAction(const std::string &action);
  static bool hasRequiredFields(const nlohmann::json &obj, const std::vector<std::string> &fields);

  // 参数类型验证
  static bool validateParameterType(const nlohmann::json &param);
  static bool validateCoordinate(const nlohmann::json &coord);
  static bool validateWaypoints(const nlohmann::json &waypoints);
};

// ========== Implementation ==========


bool TaskValidator::validateTaskFormat(const nlohmann::json &task) {
  try {
    // 检查基本结构
    if (!task.contains("stage")) {
      txtLog().error(THISMODULE "Task missing 'stage' field");
      return false;
    }

    if (!task["stage"].is_array()) {
      txtLog().error(THISMODULE "Task 'stage' field must be array");
      return false;
    }

    if (task["stage"].empty()) {
      txtLog().error(THISMODULE "Task 'stage' array is empty");
      return false;
    }

    return true;

  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Task format validation error: %s", e.what());
    return false;
  }
}

bool TaskValidator::validateStage(const nlohmann::json &stage) {
  try {
    // 检查必需字段
    std::vector<std::string> required_fields = {"name", "sn", "cmd", "actions"};
    if (!hasRequiredFields(stage, required_fields)) {
      return false;
    }

    // 验证命令
    std::string cmd = stage["cmd"].get<std::string>();
    if (!isValidCommand(cmd)) {
      txtLog().error(THISMODULE "Invalid command: %s", cmd.c_str());
      return false;
    }

    // 验证阶段编号
    int sn = stage["sn"].get<int>();
    if (sn < 0) {
      txtLog().error(THISMODULE "Invalid stage number: %d", sn);
      return false;
    }

    // 验证actions
    if (!stage["actions"].is_array()) {
      txtLog().error(THISMODULE "Stage 'actions' must be array");
      return false;
    }

    return true;

  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Stage validation error: %s", e.what());
    return false;
  }
}

bool TaskValidator::validateAction(const nlohmann::json &action) {
  try {
    // 检查必需字段
    std::vector<std::string> required_fields = {"name", "id", "groupid"};
    if (!hasRequiredFields(action, required_fields)) {
      return false;
    }

    // 验证动作名称
    std::string action_name = action["name"].get<std::string>();
    if (!isValidAction(action_name)) {
      txtLog().error(THISMODULE "Invalid action: %s", action_name.c_str());
      return false;
    }

    // 验证ID
    int id = action["id"].get<int>();
    if (id <= 0) {
      txtLog().error(THISMODULE "Invalid vehicle ID: %d", id);
      return false;
    }

    // 验证组ID
    int group_id = action["groupid"].get<int>();
    if (group_id < 0) {
      txtLog().error(THISMODULE "Invalid group ID: %d", group_id);
      return false;
    }

    // 验证参数（如果存在）
    if (action.contains("params") && !validateParameters(action["params"])) {
      return false;
    }

    // 验证触发器（如果存在）
    if (action.contains("triggers") && !validateTriggers(action["triggers"])) {
      return false;
    }

    return true;

  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Action validation error: %s", e.what());
    return false;
  }
}

bool TaskValidator::validateParameters(const nlohmann::json &params) {
  try {
    if (!params.is_array()) {
      txtLog().error(THISMODULE "Parameters must be array");
      return false;
    }

    for (const auto &param : params) {
      if (!validateParameterType(param)) {
        return false;
      }

      // 特定参数验证
      if (param.contains("name") && param.contains("value")) {
        std::string param_name = param["name"].get<std::string>();

        if (param_name == "wayPoints" || param_name == "areaPoints" || param_name == "dstLoc") {
          if (!validateWaypoints(param["value"])) {
            return false;
          }
        }

        if (param_name == "formOffset") {
          if (!validateCoordinate(param["value"])) {
            return false;
          }
        }
      }
    }

    return true;

  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Parameters validation error: %s", e.what());
    return false;
  }
}

bool TaskValidator::validateTriggers(const nlohmann::json &triggers) {
  try {
    if (!triggers.is_array()) {
      txtLog().error(THISMODULE "Triggers must be array");
      return false;
    }

    for (const auto &trigger : triggers) {
      std::vector<std::string> required_fields = {"name", "type", "value"};
      if (!hasRequiredFields(trigger, required_fields)) {
        return false;
      }

      std::string trigger_type = trigger["type"].get<std::string>();
      if (trigger_type != "string" && trigger_type != "float" &&
          trigger_type != "int" && trigger_type != "bool") {
        txtLog().error(THISMODULE "Invalid trigger type: %s", trigger_type.c_str());
        return false;
      }
    }

    return true;

  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Triggers validation error: %s", e.what());
    return false;
  }
}

bool TaskValidator::isValidCommand(const std::string &cmd) {
  return valid_commands_.contains(cmd);
}

bool TaskValidator::isValidAction(const std::string &action) {
  return valid_actions_.contains(action);
}

bool TaskValidator::hasRequiredFields(const nlohmann::json &obj, const std::vector<std::string> &fields) {
  for (const auto &field : fields) {
    if (!obj.contains(field)) {
      txtLog().error(THISMODULE "Missing required field: %s", field.c_str());
      return false;
    }
  }
  return true;
}

bool TaskValidator::validateParameterType(const nlohmann::json &param) {
  try {
    std::vector<std::string> required_fields = {"name", "type", "value"};
    if (!hasRequiredFields(param, required_fields)) {
      return false;
    }

    std::string param_type = param["type"].get<std::string>();
    if (param_type != "string" && param_type != "float" && param_type != "int" &&
        param_type != "bool" && param_type != "line" && param_type != "point" &&
        param_type != "array") {
      txtLog().error(THISMODULE "Invalid parameter type: %s", param_type.c_str());
      return false;
    }

    return true;

  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Parameter type validation error: %s", e.what());
    return false;
  }
}

bool TaskValidator::validateCoordinate(const nlohmann::json &coord) {
  try {
    if (!coord.is_object()) {
      txtLog().error(THISMODULE "Coordinate must be object");
      return false;
    }

    std::vector<std::string> required_fields = {"x_lat", "y_lon", "z_alt"};
    if (!hasRequiredFields(coord, required_fields)) {
      return false;
    }

    // 验证坐标范围
    double lat = coord["x_lat"].get<double>();
    double lon = coord["y_lon"].get<double>();

    if (lat < -90.0 || lat > 90.0) {
      txtLog().error(THISMODULE "Invalid latitude: %f", lat);
      return false;
    }

    if (lon < -180.0 || lon > 180.0) {
      txtLog().error(THISMODULE "Invalid longitude: %f", lon);
      return false;
    }

    return true;

  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Coordinate validation error: %s", e.what());
    return false;
  }
}

bool TaskValidator::validateWaypoints(const nlohmann::json &waypoints) {
  try {
    if (!waypoints.is_array()) {
      txtLog().error(THISMODULE "Waypoints must be array");
      return false;
    }

    if (waypoints.empty()) {
      txtLog().error(THISMODULE "Waypoints array is empty");
      return false;
    }
    for (const auto &waypoint : waypoints) {
      if (!validateCoordinate(waypoint)) {
        return false;
      }
    }

    return true;

  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Waypoints validation error: %s", e.what());
    return false;
  }
}