#ifndef ZYZN_GET_LOCATION_HPP
#define ZYZN_GET_LOCATION_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <custom_msgs/msg/simple_vehicle.hpp>

using namespace BT;
namespace zyzn{
    namespace get{
        /**
         * @brief 获取本机简单飞控信息
         * @details 获取自身信息，并将位置信息输出给offboard控制量
         * @author zyzn
        */
        class CGetlocation : public StatefulActionNode{
            public:
            CGetlocation(const std::string& name,
            const NodeConfig& conf);

            CGetlocation();

            static PortsList providedPorts();
            
            NodeStatus onStart(){
                return NodeStatus::RUNNING;
            }

            NodeStatus onRunning();

            void onHalted(){

            }
         
            inline static const custom_msgs::msg::SimpleVehicle & simpVehi(){
                return m_simpVehi;
            }
            private:
            void init();
            void simpVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);//精简飞控消息回调
            static custom_msgs::msg::SimpleVehicle m_simpVehi;//本机精简飞控信息
            static rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr m_s_subSelfVehi;//飞控消息订阅对象
            
            
        };
    }
}

#endif
