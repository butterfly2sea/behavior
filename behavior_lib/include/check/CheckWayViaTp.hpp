#ifndef ZYZN_CHECK_WAYVIATP_HPP
#define ZYZN_CHECK_WAYVIATP_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/dis_target.hpp>

using namespace BT;
namespace zyzn{
    namespace check{
        class CCheckWayViaTp : public SyncActionNode{
            public:
            enum{
                MAX_NAV_ID = 0xFFFFFFFF
            };
            CCheckWayViaTp(const std::string& name,
            const NodeConfig& conf);
            /**
             * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
             * arriveDis：到点距离 米
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();

            /**
             * @brief 定时执行函数，会由父节点调用
             * @return SUCCESS：任务可以开始执行，FAILED：任务还未执行
            */
            NodeStatus tick();

            inline static custom_msgs::msg::DisTarget & curNav(){
                return m_s_curNavInfo;
            }

            inline static int & preNavId(){
                return m_s_preNavId;
            }

            void wayPtDisCB(const custom_msgs::msg::DisTarget::SharedPtr msg);
            private:
            int m_accIdx;//累计航点索引
            int m_preWayIdx;  //上次完成航点索引
            rclcpp::Subscription<custom_msgs::msg::DisTarget>::SharedPtr m_subWayPt;//航点距离订阅
            static custom_msgs::msg::DisTarget m_s_curNavInfo;//当前目标航点信息
            static int m_s_preNavId;//上次目标航点索引0开始
        };
    }
}

#endif
