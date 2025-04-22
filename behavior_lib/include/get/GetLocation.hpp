#ifndef ZYZN_GET_LOCATION_HPP
#define ZYZN_GET_LOCATION_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
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

             /**
             * @brief 输入输出参数
             * @details 节点提供的输入输出参数名及类型申明
             * target：offboard控制量
             * zoffset：垂直向偏移量
             * fixed：垂直向使用固定值
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();
            
            NodeStatus onStart(){
                return NodeStatus::RUNNING;
            }

            /**
             * @brief 定时回调
             * @details 结点定时回调，将输出的x、y设置为当时位置，z值依次查看参数zoffset、fixed是否有值
             *          如有则按顺序优先采用，否则使用地面站设置值
             * @param last_msg 本机精简飞控消息
             * @result 当收到本机精简飞控消息后 总返回SUCCESS，否则返回RUNNING
            */

            NodeStatus onRunning();

            void onHalted(){

            }
         
            inline static const custom_msgs::msg::SimpleVehicle & simpVehi(){
                return m_simpVehi;
            }

            /**
             * @brief 获取飞机类型是否为固定翼
             */
            static bool isFixWing();
                
            
            private:
            void init();
            void simpVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);//精简飞控消息回调
            static custom_msgs::msg::SimpleVehicle m_simpVehi;//本机精简飞控信息
            static rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr m_s_subSelfVehi;//飞控消息订阅对象
            
        };
    }
}

#endif
