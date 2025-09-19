#include "behavior_node/utils/task_validator.hpp"
#include <log/Logger.hpp>
#include <cmath>
#include <algorithm>

namespace behavior_node {

TaskValidator::TaskValidator() {
  txtLog().debug("UTIL TaskValidator created");
}

TaskValidator::~TaskValidator() = default;

// ================================ 航线验证 ================================

bool TaskValidator::validateWaypoints(const geometry_msgs::msg::Polygon& waypoints,
                                      const WaypointValidationConfig& config) {
  try {
    if (waypoints.points.empty()) {
      txtLog().error("UTIL TaskValidator empty waypoints");
      return false;
    }

    if (waypoints.points.size() < config.min_waypoints) {
      txtLog().error("UTIL TaskValidator too few waypoints: %zu < %zu",
                     waypoints.points.size(), config.min_waypoints);
      return false;
    }

    if (waypoints.points.size() > config.max_waypoints) {
      txtLog().error("UTIL TaskValidator too many waypoints: %zu > %zu",
                     waypoints.points.size(), config.max_waypoints);
      return false;
    }

    // 验证每个航点
    for (size_t i = 0; i < waypoints.points.size(); ++i) {
      const auto& point = waypoints.points[i];

      if (!validateWaypoint(point, config)) {
        txtLog().error("UTIL TaskValidator invalid waypoint %zu", i);
        return false;
      }
    }

    // 验证航点间距
    if (config.check_waypoint_spacing) {
      if (!validateWaypointSpacing(waypoints, config)) {
        return false;
      }
    }

    txtLog().debug("UTIL TaskValidator waypoints validation passed: %zu points",
                   waypoints.points.size());
    return true;

  } catch (const std::exception& e) {
    txtLog().error("UTIL TaskValidator waypoints validation exception: %s", e.what());
    return false;
  }
}

bool TaskValidator::validateWaypoint(const geometry_msgs::msg::Point32& waypoint,
                                     const WaypointValidationConfig& config) {
  // 验证高度
  if (waypoint.z < config.min_altitude || waypoint.z > config.max_altitude) {
    txtLog().error("UTIL TaskValidator invalid waypoint altitude: %.2f not in [%.2f, %.2f]",
                   waypoint.z, config.min_altitude, config.max_altitude);
    return false;
  }

  // 验证坐标范围（可选）
  if (config.check_coordinate_bounds) {
    if (std::abs(waypoint.x) > config.max_coordinate_range ||
        std::abs(waypoint.y) > config.max_coordinate_range) {
      txtLog().error("UTIL TaskValidator waypoint out of coordinate bounds: (%.2f, %.2f)",
                     waypoint.x, waypoint.y);
      return false;
    }
  }

  // 检查是否为有效数字
  if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) || !std::isfinite(waypoint.z)) {
    txtLog().error("UTIL TaskValidator waypoint contains invalid numbers");
    return false;
  }

  return true;
}

bool TaskValidator::validateWaypointSpacing(const geometry_msgs::msg::Polygon& waypoints,
                                            const WaypointValidationConfig& config) {
  for (size_t i = 1; i < waypoints.points.size(); ++i) {
    const auto& p1 = waypoints.points[i - 1];
    const auto& p2 = waypoints.points[i];

    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < config.min_waypoint_distance) {
      txtLog().error("UTIL TaskValidator waypoints %zu-%zu too close: %.2fm < %.2fm",
                     i - 1, i, distance, config.min_waypoint_distance);
      return false;
    }

    if (distance > config.max_waypoint_distance) {
      txtLog().error("UTIL TaskValidator waypoints %zu-%zu too far: %.2fm > %.2fm",
                     i - 1, i, distance, config.max_waypoint_distance);
      return false;
    }
  }

  return true;
}

// ================================ 搜索区域验证 ================================

bool TaskValidator::validateSearchArea(const geometry_msgs::msg::Polygon& area,
                                       const SearchAreaValidationConfig& config) {
  try {
    if (area.points.empty()) {
      txtLog().error("UTIL TaskValidator empty search area");
      return false;
    }

    if (area.points.size() < config.min_vertices) {
      txtLog().error("UTIL TaskValidator search area has too few vertices: %zu < %zu",
                     area.points.size(), config.min_vertices);
      return false;
    }

    // 验证每个顶点
    for (size_t i = 0; i < area.points.size(); ++i) {
      const auto& point = area.points[i];

      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        txtLog().error("UTIL TaskValidator search area vertex %zu contains invalid numbers", i);
        return false;
      }

      // 验证高度
      if (point.z < config.min_altitude || point.z > config.max_altitude) {
        txtLog().error("UTIL TaskValidator search area vertex %zu invalid altitude: %.2f",
                       i, point.z);
        return false;
      }
    }

    // 计算并验证搜索区域面积
    if (config.check_area_size) {
      float area_size = calculatePolygonArea(area);
      if (area_size < config.min_area || area_size > config.max_area) {
        txtLog().error("UTIL TaskValidator search area size invalid: %.2f not in [%.2f, %.2f]",
                       area_size, config.min_area, config.max_area);
        return false;
      }
    }

    // 检查多边形是否自相交
    if (config.check_self_intersection && isPolygonSelfIntersecting(area)) {
      txtLog().error("UTIL TaskValidator search area has self-intersections");
      return false;
    }

    txtLog().debug("UTIL TaskValidator search area validation passed: %zu vertices",
                   area.points.size());
    return true;

  } catch (const std::exception& e) {
    txtLog().error("UTIL TaskValidator search area validation exception: %s", e.what());
    return false;
  }
}

float TaskValidator::calculatePolygonArea(const geometry_msgs::msg::Polygon& polygon) {
  if (polygon.points.size() < 3) {
    return 0.0f;
  }

  float area = 0.0f;
  size_t n = polygon.points.size();

  // 使用鞋带公式计算多边形面积
  for (size_t i = 0; i < n; ++i) {
    size_t j = (i + 1) % n;
    area += polygon.points[i].x * polygon.points[j].y;
    area -= polygon.points[j].x * polygon.points[i].y;
  }

  return std::abs(area) / 2.0f;
}

bool TaskValidator::isPolygonSelfIntersecting(const geometry_msgs::msg::Polygon& polygon) {
  size_t n = polygon.points.size();
  if (n < 4) {
    return false;  // 三角形不可能自相交
  }

  // 检查每对不相邻的边是否相交
  for (size_t i = 0; i < n; ++i) {
    size_t i_next = (i + 1) % n;

    for (size_t j = i + 2; j < n; ++j) {
      if ((j + 1) % n == i) {
        continue;  // 跳过相邻的边
      }

      size_t j_next = (j + 1) % n;

      if (doLinesIntersect(polygon.points[i], polygon.points[i_next],
                           polygon.points[j], polygon.points[j_next])) {
        return true;
      }
    }
  }

  return false;
}

bool TaskValidator::doLinesIntersect(const geometry_msgs::msg::Point32& p1,
                                     const geometry_msgs::msg::Point32& p2,
                                     const geometry_msgs::msg::Point32& p3,
                                     const geometry_msgs::msg::Point32& p4) {
  // 使用向量叉积判断线段是否相交
  auto ccw = [](const geometry_msgs::msg::Point32& A,
                const geometry_msgs::msg::Point32& B,
                const geometry_msgs::msg::Point32& C) {
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
  };

  return ccw(p1, p3, p4) != ccw(p2, p3, p4) && ccw(p1, p2, p3) != ccw(p1, p2, p4);
}

// ================================ 飞行参数验证 ================================

bool TaskValidator::validateFlightParameters(const FlightParameters& params,
                                             const FlightParameterValidationConfig& config) {
  try {
    // 验证速度
    if (params.speed < config.min_speed || params.speed > config.max_speed) {
      txtLog().error("UTIL TaskValidator invalid speed: %.2f not in [%.2f, %.2f]",
                     params.speed, config.min_speed, config.max_speed);
      return false;
    }

    // 验证高度
    if (params.altitude < config.min_altitude || params.altitude > config.max_altitude) {
      txtLog().error("UTIL TaskValidator invalid altitude: %.2f not in [%.2f, %.2f]",
                     params.altitude, config.min_altitude, config.max_altitude);
      return false;
    }

    // 验证到达距离
    if (params.arrival_distance < config.min_arrival_distance ||
        params.arrival_distance > config.max_arrival_distance) {
      txtLog().error("UTIL TaskValidator invalid arrival distance: %.2f not in [%.2f, %.2f]",
                     params.arrival_distance, config.min_arrival_distance, config.max_arrival_distance);
      return false;
    }

    // 验证循环次数
    if (config.check_loops) {
      if (params.loops < config.min_loops ||
          (config.max_loops > 0 && params.loops > config.max_loops)) {
        txtLog().error("UTIL TaskValidator invalid loops: %d not in [%d, %d]",
                       params.loops, config.min_loops, config.max_loops);
        return false;
      }
    }

    txtLog().debug("UTIL TaskValidator flight parameters validation passed");
    return true;

  } catch (const std::exception& e) {
    txtLog().error("UTIL TaskValidator flight parameters validation exception: %s", e.what());
    return false;
  }
}

// ================================ 编队参数验证 ================================

bool TaskValidator::validateFormationParameters(const FormationParameters& params,
                                                const FormationValidationConfig& config) {
  try {
    // 验证编队类型
    if (config.valid_formation_types.find(params.formation_type) ==
        config.valid_formation_types.end()) {
      txtLog().error("UTIL TaskValidator invalid formation type: %d", params.formation_type);
      return false;
    }

    // 验证成员数量
    if (params.member_ids.size() < config.min_members ||
        params.member_ids.size() > config.max_members) {
      txtLog().error("UTIL TaskValidator invalid member count: %zu not in [%zu, %zu]",
                     params.member_ids.size(), config.min_members, config.max_members);
      return false;
    }

    // 验证间距
    if (params.spacing < config.min_spacing || params.spacing > config.max_spacing) {
      txtLog().error("UTIL TaskValidator invalid formation spacing: %.2f not in [%.2f, %.2f]",
                     params.spacing, config.min_spacing, config.max_spacing);
      return false;
    }

    // 验证偏移量数量与成员数量匹配
    if (!params.offsets.points.empty() &&
        params.offsets.points.size() != params.member_ids.size()) {
      txtLog().error("UTIL TaskValidator offset count mismatch: %zu offsets for %zu members",
                     params.offsets.points.size(), params.member_ids.size());
      return false;
    }

    // 验证成员ID唯一性
    std::set<uint8_t> unique_ids(params.member_ids.begin(), params.member_ids.end());
    if (unique_ids.size() != params.member_ids.size()) {
      txtLog().error("UTIL TaskValidator duplicate member IDs detected");
      return false;
    }

    txtLog().debug("UTIL TaskValidator formation parameters validation passed");
    return true;

  } catch (const std::exception& e) {
    txtLog().error("UTIL TaskValidator formation parameters validation exception: %s", e.what());
    return false;
  }
}

// ================================ 任务阶段验证 ================================

bool TaskValidator::validateMissionStage(const std::string& stage_name,
                                         const nlohmann::json& stage_config,
                                         const MissionValidationConfig& config) {
  try {
    // 验证阶段名称
    if (config.valid_stages.find(stage_name) == config.valid_stages.end()) {
      txtLog().error("UTIL TaskValidator invalid mission stage: %s", stage_name.c_str());
      return false;
    }

    // 验证必需参数
    auto required_params_it = config.required_parameters.find(stage_name);
    if (required_params_it != config.required_parameters.end()) {
      for (const auto& required_param : required_params_it->second) {
        if (!stage_config.contains(required_param)) {
          txtLog().error("UTIL TaskValidator missing required parameter '%s' for stage '%s'",
                         required_param.c_str(), stage_name.c_str());
          return false;
        }
      }
    }

    // 特定阶段的验证逻辑
    if (stage_name == "takeoff") {
      return validateTakeoffStage(stage_config);
    } else if (stage_name == "navigation") {
      return validateNavigationStage(stage_config);
    } else if (stage_name == "search") {
      return validateSearchStage(stage_config);
    } else if (stage_name == "attack") {
      return validateAttackStage(stage_config);
    } else if (stage_name == "landing") {
      return validateLandingStage(stage_config);
    }

    txtLog().debug("UTIL TaskValidator mission stage validation passed: %s", stage_name.c_str());
    return true;

  } catch (const std::exception& e) {
    txtLog().error("UTIL TaskValidator mission stage validation exception: %s", e.what());
    return false;
  }
}

bool TaskValidator::validateTakeoffStage(const nlohmann::json& config) {
  if (config.contains("altitude")) {
    float altitude = config["altitude"];
    if (altitude <= 0.0f || altitude > 500.0f) {
      txtLog().error("UTIL TaskValidator invalid takeoff altitude: %.2f", altitude);
      return false;
    }
  }
  return true;
}

bool TaskValidator::validateNavigationStage(const nlohmann::json& config) {
  if (config.contains("waypoints") && config["waypoints"].is_array()) {
    if (config["waypoints"].empty()) {
      txtLog().error("UTIL TaskValidator navigation stage has empty waypoints");
      return false;
    }
  }

  if (config.contains("speed")) {
    float speed = config["speed"];
    if (speed <= 0.0f || speed > 50.0f) {
      txtLog().error("UTIL TaskValidator invalid navigation speed: %.2f", speed);
      return false;
    }
  }
  return true;
}

bool TaskValidator::validateSearchStage(const nlohmann::json& config) {
  if (config.contains("search_time")) {
    float search_time = config["search_time"];
    if (search_time <= 0.0f || search_time > 3600.0f) {  // 最大1小时
      txtLog().error("UTIL TaskValidator invalid search time: %.2f", search_time);
      return false;
    }
  }

  if (config.contains("target_class")) {
    int target_class = config["target_class"];
    if (target_class < 0 || target_class > 255) {
      txtLog().error("UTIL TaskValidator invalid target class: %d", target_class);
      return false;
    }
  }
  return true;
}

bool TaskValidator::validateAttackStage(const nlohmann::json& config) {
  if (config.contains("attack_mode")) {
    int attack_mode = config["attack_mode"];
    if (attack_mode < 0 || attack_mode > 10) {
      txtLog().error("UTIL TaskValidator invalid attack mode: %d", attack_mode);
      return false;
    }
  }
  return true;
}

bool TaskValidator::validateLandingStage(const nlohmann::json& config) {
  if (config.contains("landing_speed")) {
    float landing_speed = config["landing_speed"];
    if (landing_speed <= 0.0f || landing_speed > 10.0f) {
      txtLog().error("UTIL TaskValidator invalid landing speed: %.2f", landing_speed);
      return false;
    }
  }
  return true;
}

// ================================ 通用验证工具 ================================

bool TaskValidator::isValidCoordinate(const geometry_msgs::msg::Point32& point,
                                      float max_range) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
      std::abs(point.x) <= max_range && std::abs(point.y) <= max_range;
}

bool TaskValidator::isValidRange(float value, float min_value, float max_value) {
  return std::isfinite(value) && value >= min_value && value <= max_value;
}

std::string TaskValidator::getLastErrorMessage() const {
  return last_error_message_;
}

void TaskValidator::setLastErrorMessage(const std::string& message) {
  last_error_message_ = message;
  txtLog().error("UTIL TaskValidator error: %s", message.c_str());
}

}  // namespace behavior_node