//#include <behaviortree_cpp/bt_factory.h>
#ifndef ZYZN_CONTROL_LOCKCTRL_HPP
#define ZYZN_CONTROL_LOCKCTRL_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/srv/command_bool.hpp>

using namespace BT;
namespace zyzn{
    namespace ctrl{
        /**
         * @brief 上锁、解锁控制
         * @details 依据期望状态调用服务发送上锁、解锁控制
         * @author zyzn
        */
        class CLockCtrl : public StatefulActionNode{//RosServiceNode<custom_msgs::srv::CommandBool>{
            public:
            enum ELockState{
                Locked=0, ///<! 上锁状态>
                Unlock=1  ///<! 解锁状态>
            };
            CLockCtrl(const std::string& name,
            const NodeConfig& conf);

            /**
             * @brief 输入输出参数
             * @details 节点提供的输入输出参数名及类型申明
             * state：期望的锁状态
             * result：控制发送结果
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();
            /**
             * @brief 必须重载父类函数,状态为初始时调用
             * @details 判定服务是否可用，如可用返回RUNNING态
             * @return RUNNING:服务可用 IDEL:服务不可用
            */
            NodeStatus onStart();
            
            /**
             * @brief 必须重载父类函数,状态为RUNNING时调用
             * @details 判定服务是否已经为期望状态，如未达到则每隔10tick发送一次请求
             * @return RUNNING:未达到状态 SUCCESS:达到状态
            */
            NodeStatus onRunning();

            /**
             * @brief 必须重载父类函数,结束时调用
             * @details 当状态为RUNNING 框架调用halt时的回调
             * @return 无
            */
            void onHalted();
          
            private:

            static rclcpp::Client<custom_msgs::srv::CommandBool>::SharedPtr m_s_cltLockCtrl;//锁控制对象
            int m_tickCount;//tick计数,用于每隔几次tick发送一次服务
            int m_expState;//期望上锁状态用于保存输入参数值

        };
    }

}

#endif