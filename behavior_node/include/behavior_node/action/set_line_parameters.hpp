#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <custom_msgs/msg/param_short.hpp>
#include "behavior_node/common/base_enum.hpp"

// 设置线路参数节点
class SetLineParameters : public BT::SyncActionNode {
 public:
  SetLineParameters(const std::string &name,
                    const BT::NodeConfig &config,
                    std::shared_ptr<rclcpp::Node> node);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_time_;// 航线飞行开始时间发布对象
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_coll_dis_;// 防撞距离发布对象
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_arv_dis_; // 到点距离发布对象
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_grp_;// 分组发布对象
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_loops_;// 循环次数发布对象
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_spd_;// 速度发布对象
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_vehicle_typ_;// 载具类型发布对象
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_ids_;// 分组内id发布对象
  rclcpp::Publisher<custom_msgs::msg::ParamShort>::SharedPtr pub_form_;// 编队方式发布对象
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_line_;// 航点消息发布对象
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_offsets_;// 分组偏移发布对象


  SetContentTyp set_type_{SetContentTyp::ALL};// 待设置内容项类型
  int loops_{1};// 循环次数
  float arv_dis_{0.5f};// 到点距离
  std::set<uint8_t> ext_ids_;// 排除分组id
  uint8_t grp_;// 分组
  std_msgs::msg::UInt8MultiArray ids_;// 分组内id
  geometry_msgs::msg::Polygon offsets_;// 分组偏移
  geometry_msgs::msg::Polygon way_pts_;// 航线点
  float spd_{1.0f};// 速度

};