#include "behavior_node/action/set_destination_point.hpp"

#include <log/Logger.hpp>

#include <custom_msgs/msg/offboard_ctrl.hpp>

#include "behavior_node/common/data_manager.hpp"
#include "behavior_node/common/utility.hpp"

SetDestinationPoint::SetDestinationPoint(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) {
}

BT::PortsList SetDestinationPoint::providedPorts() {
  return {
      BT::InputPort<int>("step", 0, "当前步骤"),
      BT::InputPort<float>("obsHgh", -60.0f, "障碍高度"),
      // TODO:: 为啥不改为float？貌似以下input port的类型都应该改为float
      BT::InputPort<std::string>("rdsParam"),// 半径
      // 以下input port没有在代码中使用到
      BT::InputPort<std::string>("itvParam"),// 间隔
      BT::InputPort<std::string>("altParam"),// 高度
      BT::InputPort<std::string>("ptTypParam"),// 点类型
      BT::InputPort<std::string>("dstParam"),// 目标点
      BT::OutputPort<custom_msgs::msg::OffboardCtrl>("target", "目标位置")
  };
}

BT::NodeStatus SetDestinationPoint::tick() {
  custom_msgs::msg::OffboardCtrl target_msg;
  target_msg.ordmask = OffBoardMask::LocCtrl + OffBoardMask::YawCtrl;
  // 获取目标点
  auto point_json = DataManager::getInstance().getJsonParams("dstParam");
  geometry_msgs::msg::Point32 destination_point;
  if (point_json.is_array() && point_json.size() > 0) {
    // TODO:: 为什么只取第一个点
    if (auto point = point_json[0];point.contains("x") && point.contains("y") && point.contains("z")) {
      destination_point.x = point["x"].get<float>();
      destination_point.y = point["y"].get<float>();
      destination_point.z = point["z"].get<float>();
      // 检查高度有效性
      utility::checkZValid(destination_point.z);
      // 如果是gps点，则转换为地图坐标
      if (!(DataManager::getInstance().getJsonParams("pointTag").get<std::string>() == "loc")) {
        utility::gps2loc(DataManager::getInstance().getHome(), destination_point);
      }
    }
  }
  target_msg.x = destination_point.x;
  target_msg.y = destination_point.y;
  target_msg.z = destination_point.z;

  BT::Expected<int> step = getInput<int>("step");
  BT::Expected<float> obs_hgh = getInput<float>("obsHgh");
  if (step && obs_hgh) {
    auto simple_vehicle = DataManager::getInstance().getSimpleVehicle();
    if (step.value() == EStep::CurHorObsHgh) {
      // 当前水平位置 障碍高度
      target_msg.x = simple_vehicle.x / 1e3;
      target_msg.y = simple_vehicle.y / 1e3;
      target_msg.z = obs_hgh.value();
    } else if (step.value() == EStep::DstHorObsHgh) {
      // 目标水平位置 障碍高度
      target_msg.z = obs_hgh.value();
    } else if (step.value() == EStep::DstHorCurHgh) {
      // 目标水平位置 当前高度
      target_msg.z = simple_vehicle.z / 1e3;
    }
    utility::checkZValid(target_msg.z);
    // 如果是固定翼，则设置mask为loc+空速，且空速值为0飞控会使用最小空速
    if (DataManager::getInstance().getVehicleType() == VehicleType::FixWing) {
      target_msg.ordmask = OffBoardMask::FixLocRadCtrl;
      target_msg.airspd = 0.0f;
      double rds = 60.0;
      if (auto rds_input = getInput<std::string>("rdsParam"); rds_input) {
        rds = std::stof(rds_input.value());
        txtLog().info(THISMODULE "use radius via input port is:%f", rds);
      } else if (auto rds_param = DataManager::getInstance().getJsonParams("rdsParam"); rds_param.is_number()) {
        rds = rds_param.get<double>();
        txtLog().info(THISMODULE "use radius via json is:%f", rds);
      }
      target_msg.vy = rds;
    } else { // 当为旋翼时则需要计算当前位置和目的位置的航向
      // 如相距很近则使用当前航向
      if (utility::getDisFrmLoc(simple_vehicle.x / 1e3, simple_vehicle.y / 1e3, target_msg.x, target_msg.y) < 0.5) {
        target_msg.yaw = simple_vehicle.yaw / 1e3;
      } else {
        target_msg.yaw =
            utility::getYawToDest(simple_vehicle.x / 1e3, simple_vehicle.y / 1e3, target_msg.x, target_msg.y);
      }
    }
    setOutput("target", target_msg);
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