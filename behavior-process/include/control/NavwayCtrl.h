//#include <behaviortree_cpp/bt_factory.h>
#ifndef ZYZN_CONTROL_NAVWAYCTRL_HPP
#define ZYZN_CONTROL_NAVWAYCTRL_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/srv/command_int.hpp>
using namespace BT;
namespace zyzn{
    namespace ctrl{
        /**
         * @brief 编队节点控制
         * @details 对编队节点开始、暂停、继续、停止控制
         * @author zyzn
        */
        class CNavwayCtrl : public StatefulActionNode{//public RosServiceNode<custom_msgs::srv::CommandInt>{
            public:
                CNavwayCtrl(const std::string & instance_name,
            const BT::NodeConfig& conf);

            /**
             * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
             * frame：1：节点自己进行offboard控制，0：节点只进行计算不进行实际控制
             * command：控制指令 0：开始，1：暂停，2：继续，3：停止
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
       
            private:
            static rclcpp::Client<custom_msgs::srv::CommandInt>::SharedPtr m_s_navCtrl;//航线控制对象
        };
    }

}

#endif