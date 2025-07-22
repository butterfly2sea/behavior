#ifndef ZYZN_SET_TRACEATTACKOBJ_HPP
#define ZYZN_SET_TRACEATTACKOBJ_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

#include <custom_msgs/msg/object_location.hpp>


using namespace BT;

namespace zyzn
{
    namespace set{

        /**
         * @brief 设置跟踪打击目标
         * @details 利用对应话题发布飞行速度设置消息，默认话题名为：inner/set/trace_attack_object
         * @author zyzn
        */
        class CSetTraceAttackObj : public SyncActionNode
        {
        public:
            enum EObjId{
                NacObj1=1, ///<! 吊舱凝视框的目标id为1>
                NacObj2=2, ///<! 吊舱自己计算的gps目标id为2>
                GrdObj=3   ///<! 地面目标id为3>
            };

            /**
             * 目标信息
            */
            struct STgtInfo{
                u_int8_t srcId;//report vehicle id
                u_int8_t tgtId;//目标id
            };
            CSetTraceAttackObj(const std::string &instance_name,
                            const BT::NodeConfig &conf);
            static PortsList providedPorts();



            /**
             * @brief 定时执行函数，会由父节点调用
             * @result 
            */
            NodeStatus tick();

            inline static custom_msgs::msg::ObjectLocation & objLoc(){
                return m_s_attckObj;
            }
            inline static STgtInfo & tgtInfo(){
                return m_s_tgt;
            }

            private:
            static custom_msgs::msg::ObjectLocation m_s_attckObj;//目标信息
            static STgtInfo m_s_tgt;//任务打击目标信息
            rclcpp::Publisher<custom_msgs::msg::ObjectLocation>::SharedPtr m_pubTraceAttackObj;
        };
    }
}
#endif