#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behavior_node/data/base_enum.hpp"

#include "behavior_node/base_nodes.hpp"

// 设置线路参数节点
class SetLineParameters : public SyncActionBase<SetLineParameters> {
 public:
  SetLineParameters(const std::string &name, const BT::NodeConfig &config, NodeDependencies deps)
      : SyncActionBase<SetLineParameters>(name, config, deps) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<int>("type",
                           1023,
                           "设置类型掩码 (ALL=1023, VEHICLE_TYP=1, SPD=2, ANTI_DIS=4, ARV_DIS=8, FORM=16, GROUP=32, WAY_PTS=64, LOOPS=128, IDS=256, OFFSETS=512)"),
        BT::InputPort<float>("antiDis", 5.0f, "防撞距离(米)"),
        BT::InputPort<std::string>("vehiTypParam", "vehiType", "载具类型参数名称"),
        BT::InputPort<std::string>("spdParam", "spd", "速度参数名称"),
        BT::InputPort<std::string>("disParam", "arvDis", "到达距离参数名称"),
        BT::InputPort<std::string>("ptTypParam", "pointTag", "点类型参数名称"),
        BT::InputPort<std::string>("ptsParam", "wayPoints", "航点参数名称"),
        BT::InputPort<std::string>("lpsParam", "loops", "循环次数参数名称")
    };
  }

  BT::NodeStatus tick() override;

 private:

};