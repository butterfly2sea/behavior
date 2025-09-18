#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <unordered_set>
#include <functional>

#include <log/Logger.hpp>

/**
 * 任务验证器 - 验证地面站发送的任务指令格式和内容
 */
class TaskValidator {
 private:
  static std::unordered_set<std::string> valid_commands_;
  static std::unordered_set<std::string> valid_actions_;
  static std::unordered_set<std::string> required_params_;

 public:
  TaskValidator();

  // ============= 主要验证接口 =============

  /**
   * 验证地面站指令格式
   * @param command 地面站指令JSON
   * @return 是否有效
   */
  static bool validateGroundStationCommand(const nlohmann::json &command);

  /**
   * 验证参数格式和内容
   * @param params 参数列表
   * @return 是否有效
   */
  static bool validateParameters(const nlohmann::json &params);

  /**
   * 验证动作名称
   * @param action 动作名称
   * @return 是否有效
   */
  static bool validateAction(const std::string &action);

  /**
   * 验证指令类型
   * @param cmd 指令类型
   * @return 是否有效
   */
  static bool validateCommand(const std::string &cmd);

  // ============= 辅助验证方法 =============

  /**
   * 检查JSON对象是否包含必需字段
   * @param obj JSON对象
   * @param fields 必需字段列表
   * @return 是否包含所有必需字段
   */
  static bool hasRequiredFields(const nlohmann::json &obj, const std::vector<std::string> &fields);

  /**
   * 验证参数类型
   * @param param 参数对象
   * @return 是否有效
   */
  static bool validateParameterType(const nlohmann::json &param);

  /**
   * 验证坐标格式
   * @param coord 坐标对象
   * @return 是否有效
   */
  static bool validateCoordinate(const nlohmann::json &coord);

  /**
   * 验证航点列表
   * @param waypoints 航点列表
   * @return 是否有效
   */
  static bool validateWaypoints(const nlohmann::json &waypoints);

  /**
   * 验证编队偏移
   * @param offsets 编队偏移列表
   * @return 是否有效
   */
  static bool validateFormationOffsets(const nlohmann::json &offsets);

  /**
   * 验证搜索区域
   * @param area 搜索区域
   * @return 是否有效
   */
  static bool validateSearchArea(const nlohmann::json &area);

  // ============= 数值范围验证 =============

  /**
   * 验证速度范围
   * @param speed 速度值
   * @return 是否在有效范围内
   */
  static bool validateSpeed(float speed);

  /**
   * 验证高度范围
   * @param altitude 高度值
   * @return 是否在有效范围内
   */
  static bool validateAltitude(float altitude);

  /**
   * 验证距离范围
   * @param distance 距离值
   * @return 是否在有效范围内
   */
  static bool validateDistance(float distance);

  /**
   * 验证角度范围
   * @param angle 角度值(度)
   * @return 是否在有效范围内
   */
  static bool validateAngle(float angle);

  /**
   * 验证百分比范围
   * @param percentage 百分比值
   * @return 是否在有效范围内
   */
  static bool validatePercentage(float percentage);

  // ============= 复杂验证方法 =============

  /**
   * 验证任务逻辑一致性
   * @param command 完整的任务指令
   * @return 是否逻辑一致
   */
  static bool validateTaskLogic(const nlohmann::json &command);

  /**
   * 验证飞机ID和分组ID的有效性
   * @param vehicle_id 飞机ID
   * @param group_id 分组ID
   * @return 是否有效
   */
  static bool validateVehicleAndGroupId(uint8_t vehicle_id, uint8_t group_id);

  /**
   * 验证时间戳格式
   * @param timestamp 时间戳字符串
   * @return 是否有效
   */
  static bool validateTimestamp(const std::string &timestamp);

  /**
   * 验证编队配置
   * @param formation_config 编队配置
   * @return 是否有效
   */
  static bool validateFormationConfig(const nlohmann::json &formation_config);

  /**
   * 验证搜索配置
   * @param search_config 搜索配置
   * @return 是否有效
   */
  static bool validateSearchConfig(const nlohmann::json &search_config);

  /**
   * 验证攻击配置
   * @param attack_config 攻击配置
   * @return 是否有效
   */
  static bool validateAttackConfig(const nlohmann::json &attack_config);

  // ============= 错误报告 =============

  /**
   * 获取最后的验证错误信息
   * @return 错误信息字符串
   */
  static std::string getLastError();

  /**
   * 获取详细的验证报告
   * @param command 被验证的指令
   * @return 验证报告
   */
  static std::string getValidationReport(const nlohmann::json &command);

  /**
   * 重置错误状态
   */
  static void resetErrorState();

 private:
  // ============= 内部验证方法 =============

  /**
   * 初始化有效值集合
   */
  static void initializeValidSets();

  /**
   * 设置错误信息
   * @param error 错误信息
   */
  static void setError(const std::string &error);

  /**
   * 验证数值是否在指定范围内
   * @param value 数值
   * @param min_val 最小值
   * @param max_val 最大值
   * @param name 数值名称(用于错误信息)
   * @return 是否有效
   */
  static bool validateRange(float value, float min_val, float max_val, const std::string &name);

  /**
   * 验证字符串是否非空
   * @param str 字符串
   * @param name 字符串名称(用于错误信息)
   * @return 是否有效
   */
  static bool validateNonEmpty(const std::string &str, const std::string &name);

  /**
   * 验证整数是否为正数
   * @param value 整数值
   * @param name 数值名称(用于错误信息)
   * @return 是否有效
   */
  static bool validatePositive(int value, const std::string &name);

  // ============= 静态成员 =============

  static std::string last_error_;
  static bool is_initialized_;
};

// ============= 常量定义 =============

namespace validation_constants {

// 数值范围限制
constexpr float MIN_SPEED = 0.1f;        // 最小速度 m/s
constexpr float MAX_SPEED = 30.0f;       // 最大速度 m/s
constexpr float MIN_ALTITUDE = -100.0f;  // 最小高度 m
constexpr float MAX_ALTITUDE = 2000.0f;  // 最大高度 m
constexpr float MIN_DISTANCE = 0.1f;     // 最小距离 m
constexpr float MAX_DISTANCE = 10000.0f; // 最大距离 m

// ID范围限制
constexpr uint8_t MIN_VEHICLE_ID = 1;
constexpr uint8_t MAX_VEHICLE_ID = 100;
constexpr uint8_t MIN_GROUP_ID = 0;     // 0表示无分组
constexpr uint8_t MAX_GROUP_ID = 10;

// 时间限制
constexpr int MIN_TIMEOUT = 1;          // 秒
constexpr int MAX_TIMEOUT = 3600;       // 秒

// 数组大小限制
constexpr size_t MAX_WAYPOINTS = 100;
constexpr size_t MAX_GROUP_MEMBERS = 20;
constexpr size_t MAX_PARAMETERS = 50;

} // namespace validation_constants
