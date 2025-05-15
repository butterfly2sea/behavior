#ifndef ZYZN_GET_TRACECTRL_HPP
#define ZYZN_GET_TRACECTRL_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/object_computation.hpp>
#include <behavior_lib/info/OffboardInfo.hpp>
using namespace BT;
namespace zyzn{
    namespace get{
        /**
         * @brief 获取单机本机指定目标跟踪时的offboard控制量，依据目标id和解算的目标位置信息
         * @author zyzn
        */
        class GetTraceCtrl  : public SyncActionNode{
            public:
            GetTraceCtrl(const std::string& name,
            const NodeConfig& conf);

             /**
             * @brief 输入输出参数 节点提供的输入输出参数名及类型申明
             * target：offboard控制量，输出
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();
            
            /**
             * @brief 定时执行，周期性调用tick函数
             */
            NodeStatus tick();

            private:
            void objCB(const custom_msgs::msg::ObjectComputation::SharedPtr msg);//目标位置解算消息回调
            custom_msgs::msg::OffboardCtrl m_ctrl;//输出的offboard控制量
            rclcpp::Subscription<custom_msgs::msg::ObjectComputation>::SharedPtr m_objSub;//目标位置解算消息订阅
            std::chrono::milliseconds m_lostLen;//判定目标丢失时间
            std::chrono::system_clock::time_point m_preTs;//上一次目标消息时间
            
        };
    }
}

#endif
