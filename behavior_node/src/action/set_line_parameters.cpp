#include "behavior_node/action/set_line_parameters.hpp"

#include <log/Logger.hpp>

#include "behavior_node/common/data_manager.hpp"
#include "behavior_node/common/utility.hpp"

SetLineParameters::SetLineParameters(const std::string &name,
                                     const BT::NodeConfig &config,
                                     std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), node_(node) {
  pub_line_ = node->create_publisher<geometry_msgs::msg::Polygon>("inner/set/navline", 1);
  pub_coll_dis_ = node->create_publisher<std_msgs::msg::Float32>("inner/set/dis_anti_collide", 1);
  pub_arv_dis_ = node->create_publisher<std_msgs::msg::Float32>("inner/set/dis_arrive", 1);
  pub_form_ = node->create_publisher<custom_msgs::msg::ParamShort>("inner/set/form", 1);
  pub_grp_ = node->create_publisher<std_msgs::msg::UInt8>("inner/set/group", 1);
  pub_loops_ = node->create_publisher<std_msgs::msg::Int32>("inner/set/line_loops", 1);
  pub_spd_ = node->create_publisher<std_msgs::msg::Float32>("inner/set/form_spd", 1);
  pub_vehicle_typ_ = node->create_publisher<std_msgs::msg::UInt8>("inner/set/vehicle_type", 1);
  pub_ids_ = node->create_publisher<std_msgs::msg::UInt8MultiArray>("inner/information/group_ids", 1);
  pub_offsets_ = node->create_publisher<geometry_msgs::msg::Polygon>("inner/information/group_offset", 1);
}

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
  if (!getInput("type", set_type_)) {
    txtLog().error(THISMODULE "Missing 'type' input");
    return BT::NodeStatus::FAILURE;
  }

  // 发布防撞距离
  if (ANTI_DIS & set_type_) {
    BT::Expected<float> anti_dis = getInput<float>("antiDis");
    if (!anti_dis) {
      txtLog().error(THISMODULE "Missing 'antiDis' input");
      return BT::NodeStatus::FAILURE;
    }
    std_msgs::msg::Float32 msg;
    msg.data = anti_dis.value();
    pub_coll_dis_->publish(msg); // 发布防撞距离
  }

  // 发布循环次数
  loops_ = DataManager::getInstance().getJsonParams("loops").get<int>();
  if (LOOPS & set_type_) {
    std_msgs::msg::Int32 msg;
    msg.data = loops_;
    pub_loops_->publish(msg); // 发布循环次数
  }

  // 发布载具类型
  if (VEHICLE_TYP & set_type_) {
    auto vehicle_type =
        DataManager::getInstance().getJsonParams("vehiType").get<std::string>() == "多旋翼" ? VehicleType::Coper
                                                                                            : VehicleType::FixWing;
    DataManager::getInstance().setVehicleType(vehicle_type);
    std_msgs::msg::UInt8 msg;
    msg.data = vehicle_type;
    pub_vehicle_typ_->publish(msg);
  }

  // 发布速度
  if (SPD & set_type_) {
    spd_ = DataManager::getInstance().getJsonParams("spd").get<float>();
    std_msgs::msg::Float32 msg;
    msg.data = spd_;
    pub_spd_->publish(msg); // 发布速度
  }

  // 发布到点距离
  if (ARV_DIS & set_type_) {
    arv_dis_ = DataManager::getInstance().getJsonParams("arvDis").get<float>();
    std_msgs::msg::Float32 msg;
    msg.data = arv_dis_;
    pub_arv_dis_->publish(msg); // 发布到点距离
  }

  // 发布航点信息,区域搜索时参数使用areaPoints，航线飞行时参数使用waypoints
  if (WAY_PTS & set_type_) {
    std::string
        wp_param_name = DataManager::getInstance().getActionName() == "SrchViaLine" ? "areaPoints" : "wayPoints";
    auto wp_json = DataManager::getInstance().getJsonParams(wp_param_name);
    if (wp_json.is_array()) {
      way_pts_.points.clear();
      way_pts_.points.reserve(wp_json.size());
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
        way_pts_.points.push_back(pt);
      }
      // 点类型为loc还是gps
      bool is_loc = DataManager::getInstance().getJsonParams("pointTag").get<std::string>() == "loc";
      // 如果点为gps，则转换为loc坐标
      if (!is_loc) utility::gps2loc(DataManager::getInstance().getHome(), way_pts_.points);
    }
    pub_line_->publish(way_pts_);
  }



  // 后续代码待actionmanager处再重新编写
  // if (FORM & set_type_) { pub_form_->publish(form_); }
  // 发布分组信息, 目前未见grp_变量被使用
  if (GROUP & set_type_) { pub_grp_->publish(grp_); }

  // 发布分组偏移信息, 目前未见offsets_变量被使用
  if (IDS & set_type_) { pub_ids_->publish(ids_); }
  if (OFFSETS & set_type_) { pub_offsets_->publish(offsets_); }

  return BT::NodeStatus::SUCCESS;

}