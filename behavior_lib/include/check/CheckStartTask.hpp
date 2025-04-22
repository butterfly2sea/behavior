#ifndef ZYZN_CHECK_START_TASK_HPP
#define ZYZN_CHECK_START_TASK_HPP

#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/param_short.hpp>
#include <rclcpp/time.hpp>
#include <time.h>

using namespace BT;
namespace zyzn{
    namespace check{
        /**
         * 触发类型
        */
        enum ETriggerType{
            FixTime=0,  ///<! 固定时间>
            DelayTime,  ///<! 延迟>
            Manual,     ///<! 手动>
            Auto        ///<! 自动>
        };

        /**
         * @brief 检测任务是否开始
         * 
        */
        class CCheckStartTask : public BT::SyncActionNode{
            public:
            CCheckStartTask(const std::string& name, const NodeConfig& config);
            
            /**
             * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
             * delay：延迟时间 毫秒
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();

            /**
             * @brief 设置触发类型
             * @param typ：类型
            */
            inline void setTriggerType(ETriggerType typ){
                m_type = typ;
            }

            /**
             * @brief 设置触发时间或延迟时间
             * @param t：时间 秒
            */
            inline void setTriggerTime(int64_t t){
                m_time = t;
            
            }

            /**
             * @brief 确认收到回复
            */
            inline void rcvAck(){
                m_rcvAck = true;
            }             

            /**
             * @brief 定时执行函数，会由父节点调用
             * @brief SUCCESS：任务可以开始执行，FAILED：任务还未执行
            */
            NodeStatus tick();
        private:
            ETriggerType m_type;//触发类型
            uint32_t m_time;    //触发的固定时间，当类型为固定时间触发时为系统时间(秒),当为延迟执行时为延迟时间
            time_t m_start;     //延迟执行时的开始时间
            bool m_rcvAck;      //手动执行时是否收到的确认信息
        };
    }
}

#endif


