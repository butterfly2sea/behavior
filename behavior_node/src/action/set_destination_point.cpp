#include "behavior_node/action/set_destination_point.hpp"

#include <log/Logger.hpp>

#include "behavior_node/data/base_enum.hpp"
#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/utils/utility.hpp"

BT::NodeStatus SetDestinationPoint::tick() {
  txtLog().info(THISMODULE "set destination point tick");
  custom_msgs::msg::OffboardCtrl target_msg;
  target_msg.ordmask = OffBoardMask::LocCtrl + OffBoardMask::YawCtrl;
  // 获取目标点
  auto point_json = context()->getParameter("dstLoc");
  geometry_msgs::msg::Point32 destination_point;
  if (point_json.is_array() && !point_json.empty()) {
    // TODO:: 为什么只取第一个点
    if (auto point = point_json[0];point.contains("x_lat") && point.contains("y_lon") && point.contains("z_alt")) {
      destination_point.x = point["x_lat"].get<float>();
      destination_point.y = point["y_lon"].get<float>();
      destination_point.z = point["z_alt"].get<float>();
      // 检查高度有效性
      utility::checkZValid(destination_point.z);
      // 如果是gps点，则转换为地图坐标
      if (!(context()->getParameter("pointTag").get<std::string>() == "loc")) {
        utility::gps2loc(context()->getHomePoint(), destination_point);
      }
    }
  }
  target_msg.x = destination_point.x;
  target_msg.y = destination_point.y;
  target_msg.z = destination_point.z;

  BT::Expected<int> step = getInput<int>("step");
  BT::Expected<float> obs_hgh = getInput<float>("obsHgh");
  if (step && obs_hgh) {
//    if (!cache()->isVehicleStateValid())
//      return BT::NodeStatus::FAILURE;
    auto simple_vehicle = cache()->getVehicleState();
    if (step.value() == EStep::CurHorObsHgh) {
      // 当前水平位置 障碍高度
      target_msg.x = simple_vehicle->x / 1e3;
      target_msg.y = simple_vehicle->y / 1e3;
      target_msg.z = obs_hgh.value();
    } else if (step.value() == EStep::DstHorObsHgh) {
      // 目标水平位置 障碍高度
      target_msg.z = obs_hgh.value();
    } else if (step.value() == EStep::DstHorCurHgh) {
      // 目标水平位置 当前高度
      target_msg.z = simple_vehicle->z / 1e3;
    }
    utility::checkZValid(target_msg.z);
    // 如果是固定翼，则设置mask为loc+空速，且空速值为0飞控会使用最小空速
    if (*cache()->getVehicleType() == VehicleType::FixWing) {
      target_msg.ordmask = OffBoardMask::FixLocRadCtrl;
      target_msg.airspd = 0.0f;
      double radius = 60.0; // 默认半径为60m
      if (getInput<double>("radius", radius)) {
        txtLog().info(THISMODULE "use radius via inputport is:%f", radius);
      } else if (auto dist = context()->getArrivalDistance();dist > 0.0) {
        radius = dist;
        txtLog().info(THISMODULE "use radius via arrive distance is:%f", radius);
      } else {
        txtLog().info(THISMODULE "use default radius:%f", radius);
      }
      target_msg.vy = radius;
    } else { // 当为旋翼时则需要计算当前位置和目的位置的航向
      // 如相距很近则使用当前航向
      if (utility::getDisFrmLoc(simple_vehicle->x / 1e3, simple_vehicle->y / 1e3, target_msg.x, target_msg.y) <
          0.5) {
        target_msg.yaw = simple_vehicle->yaw / 1e3;
      } else {
        target_msg.yaw = utility::getYawToDest(simple_vehicle->x / 1e3, simple_vehicle->y / 1e3, target_msg.x,
                                               target_msg.y);
      }
    }
    txtLog().info(THISMODULE
                  "set target point to (%f, %f, %f) with yaw:%f",
                  target_msg.x,
                  target_msg.y,
                  target_msg.z,
                  target_msg.yaw);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}