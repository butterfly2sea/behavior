#ifndef ZYZN_SET_TRIGGER_HPP
#define ZYZN_SET_TRIGGER_HPP
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "../info/Param.hpp"
using namespace BT;
namespace zyzn{
    namespace set{

        /**
         * @brief 设置延迟时间，延迟输出到delay参数，主要用于设置返航的延迟时间
         * @author zyzn
        */
        class CSetTrigger : public BT::SyncActionNode{
            public:
            /**
             * 任务触发类型
            */
            enum ETriggerType{
                FixedTime, ///<!固定时间>
                DelayTime, ///<!延迟>
                Manual,    ///<!手动>
                Auto
            };
            
            CSetTrigger(const std::string& name, const NodeConfig& config);
   
            /**
             * @brief 输入输出参数，节点提供的输入输出参数名及类型申明
             * delay：延迟时间
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();
            
            /**
             * @brief 定时回调，将延迟时间赋给输出参数
             * @result 总是返回SUCCESS
            */
            NodeStatus tick();
        };
    }
}

#endif

