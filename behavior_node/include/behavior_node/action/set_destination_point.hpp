#pragma once

#include <string>

#include <rclcpp/node.hpp>
#include <behaviortree_cpp/action_node.h>

#include <geometry_msgs/msg/point32.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>

#include "behavior_node/base_nodes.hpp"

// 设置目标点节点
class SetDestinationPoint : public SyncActionBase<SetDestinationPoint> {
 public:
  SetDestinationPoint(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : SyncActionBase<SetDestinationPoint>(name, config, deps) {}

  static BT::PortsList providedPorts(){
    return {
        BT::InputPort<int>("step",
                           0,
                           "当前步骤 (0-当前水平位置障碍高度, 1-目标水平位置障碍高度, 2-目标位置, 3-目标水平位置当前高度)"),
        BT::InputPort<float>("obsHgh", -60.0f, "障碍高度(米，负值)"),
        BT::InputPort<double>("radius", 60.0, "盘旋半径(米，固定翼使用)"),
        BT::InputPort<std::string>("rdsParam", "radius", "半径参数名称"),
        BT::InputPort<std::string>("itvParam", "intval", "间隔参数名称"),
        BT::InputPort<std::string>("altParam", "alt", "高度参数名称"),
        BT::InputPort<std::string>("ptTypParam", "pointTag", "点类型参数名称"),
        BT::InputPort<std::string>("dstParam", "dstLoc", "目标点参数名称"),
        BT::InputPort<std::string>("ptsParam", "wayPoints", "航点参数名称"),
        BT::OutputPort<custom_msgs::msg::OffboardCtrl>("target", "目标位置控制指令")
    };
  }
  BT::NodeStatus tick() override;
};