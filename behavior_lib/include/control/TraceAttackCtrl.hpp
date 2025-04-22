//#include <behaviortree_cpp/bt_factory.h>
#ifndef ZYZN_CONTROL_TRACEATTACKCTRL_HPP
#define ZYZN_CONTROL_TRACEATTACKCTRL_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/srv/command_int.hpp>

using namespace BT;
namespace zyzn{
    namespace ctrl{
        /**
         * @brief 跟踪打击控制
         * @details 跟踪打击控制，同集群编队控制类似，包括开始、暂停、继续及停止控制
         * @author zyzn
        */
        class CTraceAttackCtrl : public StatefulActionNode{//public RosServiceNode<custom_msgs::srv::CommandInt>{
            public:

            enum ETraceAttack{
                OAttackST=0,     ///<! 接收地面同时打击指令>
                OAttackDT,       ///<! 接收地面分时打击指令>
                OTrace=2,        ///<! 接收地面跟踪指令>
                ITrace=0,        ///<! 给指导控制的跟踪指令>
                IAttack=1,       ///<! 给指导控制的打击指令>
                UpdateMixObj=3,  ///<! 更新融合目标>
                UpdateGrdObj=4,  ///<! 更新地面目标>   
                Suicide=0,       ///<! 自杀式攻击>
                DivAttack=1,     ///<! 俯冲式攻击>
                EleMagAccLd=5,   ///<! 电磁攻击加速降落> 
                EleMagDivLd=6,   ///<! 电磁攻击俯冲降落>    
                EleMagVerLd=7,   ///<! 电磁攻击到目的点后垂直降落>    
                Abort=255        ///<! 任务终止>        
            };

            CTraceAttackCtrl(const std::string & instance_name,
            const BT::NodeConfig& conf);
            /**
             * @brief 节点提供的输入输出参数名及类型申明
             * frame：1:节点自己进行offboard控制，0：节点只计算不控制
             * command：控制指令，0：开始，1：暂停，2：继续，3：停止
             * current: 0：跟踪，1：打击
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();

            /**
             * @brief 必须重载父类函数,状态为初始时调用
             * @return RUNNING:服务可用 IDEL:服务不可用
            */
            NodeStatus onStart();
            
            /**
             * @brief 必须重载父类函数,状态为RUNNING时调用
             * @return RUNNING:未达到状态 SUCCESS:达到状态
            */
            NodeStatus onRunning();

            /**
             * @brief 必须重载父类函数,结束时调用
             * @return 无
            */
            void onHalted();
         
            public:
            static int m_s_traceAttackType;//跟踪打击类型 0：跟踪，1：打击
            private:
            static rclcpp::Client<custom_msgs::srv::CommandInt>::SharedPtr m_s_traceAttackCtrl;//跟踪打击控制对象
        };
    }

}

#endif