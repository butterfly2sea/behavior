#pragma once
#include <string_view>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <behaviortree_cpp/basic_types.h>

static const int TickCount = 100;

/**
 * 航点类型
*/
enum class PointType {
  Loc = 0,///<!loc位置>
  Gps   ///<!gps位置>
};

/**
 * offboard控制字
*/
enum OffBoardMask {
  RollCtrl = 1,
  PitchCtrl = 2,
  YawCtrl = 4,
  ThrottleCtrl = 8,
  LocCtrl = 0x70,// 16|32|64
  VxCtrl = 128,
  VyCtrl = 256,
  VzCtrl = 512,
  VelCtrl = 0x0380,// 128|256|512,
  AttitudeCtrl = 0x0F,// 1|2|4|8,
  AirSpdCtrl = 1024,
  GpsCtrl = 0x3800,// 2048|4096|8192
  FixWingCtrl,
  FixLocRadCtrl = LocCtrl + VyCtrl + AirSpdCtrl ///<! 固定翼盘旋控制>
};

enum class YawAngleConst {
  MicRadPerDeg = 17,///<!每度的豪弧值>
  YawErrDeg = 10,   ///<!判定到达目标航向的误差,度数>
  YawErrRad = YawErrDeg * MicRadPerDeg,///<!判定到达目标航向的误差,豪弧数>
  InitYaw = 9999///<!航向初始量用于判定偏航是否设置>
};

// 参数类型
enum class ParamType {
  ALL,           // 所有参数
  ANTI_DISTANCE, // 防撞距离
  WAY_PTS,       // 航路点
  SPEED,         // 速度
  ALTITUDE       // 高度
};

enum VehicleType {
  FixWing = 1, ///<!固定翼>
  Coper = 2,    ///<!旋翼>
  VtolCop,     ///<!垂起-旋翼模式>
  VtolFix,     ///<!垂起-固定翼模式>
  Car          ///<!小车>
};

enum class FlightMode {
  Unknown = 0,   // 未知
  Manual,      // 手动
  Alt,         // 定高
  Pos,         // 定点
  TakeOff,     // 起飞
  Land,        // 降落
  Hold,        // 等待
  Mission,     // 任务
  Rtl,         // 返航
  Offboard,    //外部
  Stable       //增稳
};

enum class JoyControl {
  MinMan = -1000, ///<! 手动控制消息的最小值>
  MaxMan = 1000,  ///<! 手动控制消息的最大值>
  MinJoy = 1000,  ///<! 摇杆消息的最小值>
  MaxJoy = 2000,  ///<! 摇杆消息的最大值>
  MidJoy = 1500,  ///<! 摇杆消息的中值>
  PitchCh = 1,    ///<! 俯仰通道索引>
  RollCh = 0,     ///<! 横滚通道索引>
  ThroCh = 2,     ///<! 油门通道索引>
  YawCh = 3,      ///<! 偏航通道索引>
  InvalTs = 0,    ///<! 无限时间值>
  LostCtrlLen = 2000///<! 失去摇杆数据时长判定 毫秒>
};

enum LockState {
  LOCK = 0,    // 锁定
  UNLOCK = 1   // 解锁
};

// 命令类型
enum NavCommandType {
  START = 0,    // 开始导航
  PAUSE = 1,    // 暂停导航
  RESUME = 2,   // 恢复导航
  STOP = 3      // 停止导航
};

// 坐标系类型
enum NavFrameType {
  LOCAL_NED = 0,     // 本地坐标系
  BODY_NED = 1,      // 机体坐标系
  LOCAL_ENU = 2,     // 本地ENU坐标系
  GLOBAL = 3         // 全球坐标系
};

// off_board控制类型
enum class ControlType {
  POSITION,    // 位置控制
  VELOCITY,    // 速度控制
  ATTITUDE,    // 姿态控制
  ACCELERATION // 加速度控制
};

// set_line
enum SetContentTyp {
  VEHICLE_TYP = 1,  ///<! 载具类型>
  SPD = 2,          ///<! 速度>
  ANTI_DIS = 4,     ///<! 避撞距离>
  ARV_DIS = 8,      ///<! 到点判定>
  FORM = 16,        ///<! 编队类型>
  GROUP = 32,       ///<! 分组>
  WAY_PTS = 64,     ///<! 航向航点>
  LOOPS = 128,      ///<! 循环>
  IDS = 256,        ///<! 分组id>
  OFFSETS = 512,    ///<! 分组偏移>
  ALL = 1023,       ///<! 全部内容>
  TWO_SWITCH = IDS + OFFSETS,///<! 编队分组变化只设置同组id和偏移>
  FOUR_SWITCH = TWO_SWITCH + LOOPS + WAY_PTS///<! 编队分组变化设置同组id、偏移、循环和航点>
};

enum EStep{
  CurHorObsHgh=0,    ///<! 当前水平位置 障碍高度>
  DstHorObsHgh,      ///<! 目的水平位置 障碍高度>
  DstPos,            ///<! 目的位置>
  DstHorCurHgh       ///<! 目的水平位置 当前高度>
};

// 高度类型
enum HeightType {
  ABSOLUTE,   // 绝对高度
  RELATIVE,   // 相对高度
  FIXED       // 固定高度
};

// 任务状态
enum StatusStage{
  StsNoStart,///<!未开始>
  StsOngoing,///<!正在进行>
  StsFailed, ///<!失败>
  StsComplete,///<!完成>
  StsNotready,///<!未就绪>
  StsNull
};