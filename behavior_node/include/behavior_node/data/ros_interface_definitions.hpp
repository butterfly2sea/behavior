#pragma once

#include <rclcpp/qos.hpp>

namespace ros_interface {

// 话题名称常量
namespace topics {
// 订阅输入

constexpr const char *STAGE_JSON = "outer/set/stage_info_json"; // json任务信息
constexpr const char *COMMAND = "outer/command/request"; // 控制指令
constexpr const char *ATTACK_DESIGNATE = "outer/set/attack_object_designate"; // 目标跟踪打击
constexpr const char *VEHICLE_STATE = "inner/information/simple_vehicle"; // 精简飞控消息
constexpr const char *WAYPOINT = "inner/information/way_point"; // 当前航点及距离航点距离
constexpr const char *OBJECT_DETECTION = "inner/information/object_computation"; // 目标位置消息
constexpr const char *JOYSTICK = "inner/control/joystick"; // 遥控器消息（待补充进文档）

// 发布输出
constexpr const char *INFO_IMAGE_DISTRIBUTE = "inner/set/image_distribute"; // 视频分发设置
constexpr const char *SET_COORD = "inner/set/coord_calibration"; // 设置home点
constexpr const char *OFFBOARD_CONTROL = "inner/control/offboard"; // offboard控制指令
constexpr const char *SET_VEHICLE_TYPE = "inner/set/vehicle_type"; // 设置载具类型（旋翼、固定翼切换）
constexpr const char *OUTER_RESPONSE = "outer/command/response"; // 控制指令回复
constexpr const char *OUTER_STATUS_TASK = "outer/information/status_task"; // 任务状态信息
constexpr const char *SET_ANTI_COLLISION_DIS = "inner/set/dis_anti_collide"; // 设置避撞距离
constexpr const char *SET_NAVLINE = "inner/set/navline"; // 设置航线
constexpr const char *SET_GROUP_IDS = "inner/information/group_ids"; // 设置同分组内飞机id
constexpr const char *SET_GROUP_SPEED = "inner/set/form_spd"; // 设置编队航速
constexpr const char *SET_GROUP_OFFSET = "inner/information/group_offset"; // 设置分组内编队位置偏移
constexpr const char *SET_ARRIVAL_DIS = "inner/set/dis_arrive"; // 设置到点距离
constexpr const char *SET_LOOPS = "inner/set/line_loops"; // 设置航线飞行圈数
constexpr const char *SET_GROUP = "inner/set/group"; // 设置编队（待补充进文档）
constexpr const char *SET_FORMATION = "inner/set/form"; // 设置编队形态（待补充进文档）
constexpr const char *MANUAL_CONTROL = "/mavros/manual_control/send"; // 手动控制指令（待补充进文档）
//constexpr const char *SET_OBJECT_FILTER = "inner/set/object_filter"; // 设置目标过滤器 （目前未使用）

constexpr const char *INFO_IMAGE_CHANNEL = "inner/set/image_channel";
constexpr const char *INFO_CAMERA_CONTROL = "/multispectral_camera/control";

// mavros话题
}

// 服务名称常量
namespace services {
constexpr const char *INFO_RTSP_URL = "inner/get/rtsp_url"; // 获取RTSP流地址
constexpr const char *SET_FLIGHT_MODE = "inner/control/set_flymode"; // 设置飞行模式
constexpr const char *LOCK_UNLOCK = "inner/control/lock_unlock"; // 锁定解锁
constexpr const char *GUIDANCE_SWITCH = "inner/control/guidance_switch"; // 跟踪打击控制
constexpr const char *FORMATION_SWITCH = "inner/control/form_switch"; // 编队控制

constexpr const char *GET_SELF_ID = "inner/get/id_vehicle"; // 获取自身ID（未使用）
}

} // namespace ros_interface