#include "behavior_node/action/set_line_parameters.hpp"

#include <log/Logger.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "behavior_node/data/ros_communication_manager.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/utils/utility.hpp"

BT::PortsList SetLineParameters::providedPorts() {
  return {
      BT::InputPort<float>("antiDis", 5.0f, "防撞距离"),
      BT::InputPort<int>("type"),
      BT::InputPort<std::string>("vehiTypParam", "vehicle type"),
      BT::InputPort<std::string>("spdParam", "speed"),
      BT::InputPort<std::string>("ptsParam", "way point"),
      BT::InputPort<std::string>("disParam", "arrival distance"),
      BT::InputPort<std::string>("ptsParam", "way with points and yaw"),
      BT::InputPort<std::string>("lpsParam", "loops")
  };
}

BT::NodeStatus SetLineParameters::tick() {
  SetContentType set_type = SetContentType::ALL;
  if (!getInput("type", set_type)) {
    txtLog().error(THISMODULE "Missing 'type' input");
    return BT::NodeStatus::FAILURE;
  }

  context()->setSetType(set_type);

  // 发布防撞距离
  if (ANTI_DIS & set_type) {
    BT::Expected<float> anti_dis = getInput<float>("antiDis");
    if (!anti_dis) {
      txtLog().error(THISMODULE "Missing 'antiDis' input");
      return BT::NodeStatus::FAILURE;
    }
    std_msgs::msg::Float32 msg;
    msg.data = anti_dis.value();
    ros()->publish<std_msgs::msg::Float32>(ros_interface::topics::SET_ANTI_COLLISION_DIS, msg); // 发布防撞距离
  }

  // 发布循环次数
  int loops = context()->getParameter("loops").get<int>();
  if (LOOPS & set_type) {
    std_msgs::msg::Int32 msg;
    msg.data = loops;
    ros()->publish<std_msgs::msg::Int32>(ros_interface::topics::SET_LOOPS, msg); // 发布循环次数
  }

  // 发布载具类型
  if (VEHICLE_TYP & set_type) {
    auto vehicle_type =
        context()->getParameter("vehiType").get<std::string>() == "多旋翼" ? VehicleType::Coper : VehicleType::FixWing;
    cache()->updateVehicleType(vehicle_type);
    std_msgs::msg::UInt8 msg;
    msg.data = vehicle_type;
    ros()->publish<std_msgs::msg::UInt8>(ros_interface::topics::SET_VEHICLE_TYPE, msg); // 发布载具类型
  }

  // 发布速度
  if (SPD & set_type) {
    float speed = context()->getParameter("speed").get<float>();
    std_msgs::msg::Float32 msg;
    msg.data = speed;
    ros()->publish<std_msgs::msg::Float32>(ros_interface::topics::SET_SPEED, msg); // 发布速度
  }

  // 发布到点距离
  if (ARV_DIS & set_type) {
    float arv_dis = context()->getParameter("arvDis").get<float>();
    std_msgs::msg::Float32 msg;
    msg.data = arv_dis;
    ros()->publish<std_msgs::msg::Float32>(ros_interface::topics::SET_ARRIVAL_DIS, msg); // 发布到点距离
  }

  // 发布航点信息,区域搜索时参数使用areaPoints，航线飞行时参数使用waypoints
  if (WAY_PTS & set_type) {
    geometry_msgs::msg::Polygon way_points;
    std::string wp_param_name = context()->getAction() == "SrchViaLine" ? "areaPoints" : "wayPoints";
    auto wp_json = context()->getParameter(wp_param_name);
    if (wp_json.is_array()) {
      way_points.points.clear();
      way_points.points.reserve(wp_json.size());
      for (auto &wp_json_item : wp_json) {
        // 如果点符合格式，则添加到航点列表
        if (!wp_json_item.is_object() || !wp_json_item.contains("x_lat") || !wp_json_item.contains("y_lon")
            || !wp_json_item.contains("z_alt")) {
          continue;
        }
        geometry_msgs::msg::Point32 pt;
        pt.x = wp_json_item["x_lat"].get<float>();
        pt.y = wp_json_item["y_lon"].get<float>();
        pt.z = wp_json_item["z_alt"].get<float>();
        utility::checkZValid(pt.z);
        way_points.points.push_back(pt);
      }
      // 点类型为loc还是gps
      bool is_loc = context()->getParameter("pointTag").get<std::string>() == "loc";
      // 如果点为gps，则转换为loc坐标
      if (!is_loc) utility::gps2loc(context()->getHomePoint(), way_points.points);
    }
    ros()->publish<geometry_msgs::msg::Polygon>(ros_interface::topics::SET_NAVLINE, way_points); // 发布航点信息
  }



  // 后续代码待actionmanager处再重新编写
  // if (FORM & set_type) { pub_form_->publish(form_); }
  // 发布分组信息, 目前未见grp_变量被使用
//  if (GROUP & set_type) { pub_grp_->publish(grp_); }  ros_interface::topics::SET_GROUP

  // 发布分组偏移信息, 目前未见offsets_变量被使用
//  if (IDS & set_type) { pub_ids_->publish(ids_); }
//  if (OFFSETS & set_type) { pub_offsets_->publish(offsets_); }

//ros_interface::topics::SET_FORMATION

  return BT::NodeStatus::SUCCESS;

}