#ifndef ZYZN_CONTROL_OFFBOARDCTRL_HPP
#define ZYZN_CONTROL_OFFBOARDCTRL_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/offboard_ctrl.hpp>

using namespace BT;
namespace zyzn{
    namespace ctrl{
        /**
         * @brief 发送offboard控制话题,话题名默认为inner/control/offboard
         * @author zyzn
        */
        class COffboardCtrl : public SyncActionNode{
            public:
            COffboardCtrl(const std::string& name,
            const NodeConfig& conf);

            /**
             * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
             * ctrl：offboard控制量信息，由其它节点提供值
             * @result 输入输出参数列表
            */

            static PortsList providedPorts();
            /**
             * @brief 发送offboard控制消息,将offboard控制量，发送给offboard控制消息
             * @param msg in out 控制量
             * @result 是否发送，总是返回true
            */
            NodeStatus tick();
            private:
            rclcpp::Publisher<custom_msgs::msg::OffboardCtrl>::SharedPtr m_pubOffbdCtrl;


        };
    }
}

#endif