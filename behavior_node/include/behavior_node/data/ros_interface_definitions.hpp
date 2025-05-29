#pragma once

#include <rclcpp/qos.hpp>

namespace ros_interface {

// 话题名称常量
namespace topics {
// 控制命令
constexpr const char *OFFBOARD_CONTROL = "inner/control/offboard";
constexpr const char *JOYSTICK = "inner/control/joystick";

// 参数设置
constexpr const char *SET_NAVLINE = "inner/set/navline";
constexpr const char *SET_ANTI_COLLISION_DIS = "inner/set/dis_anti_collide";
constexpr const char *SET_ARRIVAL_DIS = "inner/set/dis_arrive";
constexpr const char *SET_GROUP = "inner/set/group";
constexpr const char *SET_LOOPS = "inner/set/line_loops";
constexpr const char *SET_SPEED = "inner/set/form_spd";
constexpr const char *SET_VEHICLE_TYPE = "inner/set/vehicle_type";
constexpr const char *SET_COORD = "inner/set/coord_calibration"; // TODO:: topic名为坐标校准，但是代码中用于设置home，待确认
constexpr const char *SET_FORMATION = "inner/set/form";

// 内部信息
constexpr const char *INFO_IMAGE_DISTRIBUTE = "inner/set/image_distribute";
constexpr const char *INFO_IMAGE_CHANNEL = "inner/set/image_channel";
constexpr const char *INFO_CAMERA_CONTROL = "/multispectral_camera/control";
constexpr const char *INFO_VEHICLE_STATE = "inner/information/simple_vehicle";
constexpr const char *INFO_OBJECT_DETECTION = "inner/information/object_computation";
constexpr const char *INFO_WAYPOINT = "inner/information/way_point";
constexpr const char *INFO_GPIO_STATUS = "inner/information/gpio_status";
constexpr const char *INFO_GROUP_IDS = "inner/information/group_ids";
constexpr const char *INFO_GROUP_OFFSET = "inner/information/group_offset";

// 外部接口
constexpr const char *OUTER_STATUS_TASK = "outer/information/status_task";
constexpr const char *OUTER_VEHICLE_STATE = "outer/information/simple_vehicle";
constexpr const char *OUTER_STAGE_JSON = "outer/set/stage_info_json";
constexpr const char *OUTER_COMMAND = "outer/command/request";
constexpr const char *OUTER_TASK_STAGE = "outer/set/task_stage";
constexpr const char *OUTER_ATTACK_DESIGNATE = "outer/set/attack_object_designate";
constexpr const char *OUTER_RESPONSE = "outer/command/response";

// mavros话题
constexpr const char *MANUAL_CONTROL = "/mavros/manual_control/send";
}

// 服务名称常量
namespace services {
constexpr const char *SET_FLIGHT_MODE = "inner/control/set_flymode";
constexpr const char *LOCK_UNLOCK = "inner/control/lock_unlock";
constexpr const char *FORMATION_SWITCH = "inner/control/form_switch";
constexpr const char *INFO_RTSP_URL = "inner/get/rtsp_url";

// 补充的服务
constexpr const char *ARM_DISARM = "inner/control/arm_disarm";
constexpr const char *EMERGENCY_STOP = "inner/control/emergency_stop";
constexpr const char *MISSION_CONTROL = "inner/control/mission_control";
}

// QoS配置
namespace qos {
inline rclcpp::QoS control_commands() {
  return rclcpp::QoS(10).reliable().durability_volatile();
}

inline rclcpp::QoS sensor_data() {
  return rclcpp::SensorDataQoS();
}

inline rclcpp::QoS mission_commands() {
  return rclcpp::QoS(10).reliable().transient_local();
}

inline rclcpp::QoS status_info() {
  return rclcpp::QoS(10).best_effort();
}
}

} // namespace ros_interface