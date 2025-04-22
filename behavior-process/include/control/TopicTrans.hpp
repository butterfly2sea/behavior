#ifndef ZYZN_CONTROL_OFFBOARDCTRL_HPP
#define ZYZN_CONTROL_OFFBOARDCTRL_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/string.hpp>

using namespace BT;
namespace zyzn{
    namespace ctrl{
        /**
         * @brief 发送转发话题，话题名通过参数传入
         * @author zyzn
        */
        class TopicTrans  : public SyncActionNode{
            public:
            TopicTrans (const std::string& name,
            const NodeConfig& conf);

            /**
             * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
             * tpic：待发布话题名
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();
            
            /**
             * @brief 发送对应话题信息
             * @param msg in out 控制量
             * @result 是否发送，总是返回true
            */
            NodeStatus tick();

            inline static std::string & info(){
                return m_info.data;
            }
            private:
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubInfo;

            static std_msgs::msg::String m_info;//待发送信息
        };
    }
}

#endif