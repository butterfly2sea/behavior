#pragma once

#include <cstdint>

// 飞行模式枚举（与现有实现保持一致）
enum class FlightMode : uint8_t {
  UNKNOWN = 0,
  MANUAL = 1,
  ALTITUDE_HOLD = 2,
  POSITION_HOLD = 3,
  TAKEOFF = 4,
  LAND = 5,
  LOITER = 6,
  MISSION = 7,
  RTL = 8,
  OFFBOARD = 9,
  STABILIZE = 10
};

// 载具类型枚举
enum class VehicleType : uint8_t {
  UNKNOWN = 0,
  FIXED_WING = 1,
  MULTI_ROTOR = 2,
  VTOL = 3,
  HELICOPTER = 4
};

// 任务状态枚举
enum class StatusStage : uint8_t {
  StsNone = 0,
  StsOngoing = 1,
  StsCompleted = 2,
  StsFailed = 3,
  StsCancelled = 4
};

// 目标类别枚举
enum class TargetClass : uint8_t {
  UNKNOWN = 0,
  VEHICLE = 1,
  PERSON = 2,
  BUILDING = 3,
  SHIP = 4,
  AIRCRAFT = 5
};

// 编队模式枚举
enum class FormationMode : uint8_t {
  NONE = 0,
  LINE = 1,
  WEDGE = 2,
  VEE = 3,
  DIAMOND = 4,
  BOX = 5,
  CIRCLE = 6
};

// 搜索模式枚举
enum class SearchPattern : uint8_t {
  NONE = 0,
  LINE = 1,
  GRID = 2,
  SPIRAL = 3,
  ZIGZAG = 4,
  RANDOM = 5
};

// 攻击模式枚举
enum class AttackMode : uint8_t {
  NONE = 0,
  DIRECT = 1,
  COORDINATED = 2,
  SEQUENTIAL = 3,
  SIMULTANEOUS = 4
};

// 返航模式枚举
enum class RTLMode : uint8_t {
  DIRECT = 0,
  VIA_WAYPOINTS = 1,
  FORMATION = 2
};

// 控制类型枚举
enum class ControlType : uint8_t {
  POSITION = 0,
  VELOCITY = 1,
  ACCELERATION = 2,
  ATTITUDE = 3,
  RATE = 4
};

// 坐标系枚举
enum class CoordinateFrame : uint8_t {
  GLOBAL = 0,           // 全球坐标系
  LOCAL_NED = 1,        // 本地NED坐标系
  BODY = 2,             // 机体坐标系
  LOCAL_OFFSET_NED = 3  // 本地偏移NED坐标系
};

// GPS Fix类型枚举
enum class GPSFixType : uint8_t {
  NO_GPS = 0,
  NO_FIX = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  DGPS = 4,
  RTK_FLOAT = 5,
  RTK_FIXED = 6
};

// 电池状态枚举
enum class BatteryState : uint8_t {
  UNKNOWN = 0,
  OK = 1,
  LOW = 2,
  CRITICAL = 3,
  EMERGENCY = 4,
  FAILED = 5
};

// 武器状态枚举
enum class WeaponState : uint8_t {
  SAFE = 0,
  ARMED = 1,
  FIRING = 2,
  RELOADING = 3,
  ERROR = 4
};

// 通信状态枚举
enum class CommState : uint8_t {
  DISCONNECTED = 0,
  CONNECTING = 1,
  CONNECTED = 2,
  ERROR = 3
};

// 设置内容类型枚举（与现有实现保持一致）
enum class SetContentType : uint32_t {
  ONE_SWITCH = 1,
  TWO_SWITCH = 2,
  THREE_SWITCH = 3,
  FOUR_SWITCH = 4,
  FIVE_SWITCH = 5,
  SIX_SWITCH = 6,
  SEVEN_SWITCH = 7,
  ALL_SWITCH = 127
};

// 触发类型枚举
enum class TriggerType : uint8_t {
  NONE = 0,
  TIME = 1,
  POSITION = 2,
  ALTITUDE = 3,
  SPEED = 4,
  DISTANCE = 5,
  MANUAL = 6,
  CONDITION = 7
};

// 点类型枚举
enum class PointType : uint8_t {
  WAYPOINT = 0,
  TAKEOFF = 1,
  LAND = 2,
  LOITER = 3,
  SEARCH = 4,
  ATTACK = 5,
  RALLY = 6,
  HOME = 7
};

// 执行状态枚举
enum class ExecutionState : uint8_t {
  IDLE = 0,
  PREPARING = 1,
  EXECUTING = 2,
  PAUSED = 3,
  COMPLETED = 4,
  FAILED = 5,
  CANCELLED = 6
};