#ifndef ZYZN_CHECK_QUITSEARCH_HPP
#define ZYZN_CHECK_QUITSEARCH_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/object_computation.hpp>

using namespace BT;

namespace zyzn{
    namespace check{
        /**
         * @brief 检测是否停止搜索
         * @details 现简单依据是否收到目标位置消息进行判定，如收到则认为已经搜索到目标，可以停止搜索
         * @author
        */
        class CCheckQuitSearch : public SyncActionNode
        {
            public:
                CCheckQuitSearch(const std::string & instance_name,
                           const BT::NodeConfig& conf);

                static PortsList providedPorts();
                
                /**
                 * @brief 设置是否自动进入下个阶段
                 * @param v 是否自动
                 * @result
                */
                inline void setAutoNxt(bool v){
                    m_autoNxt = v;
                }

                /**
                 * @brief  节点定时回调,依据是否识别到目标且目标位置解算成功
                 * @return 是否可以停止搜索
                 * SUCCESS:是
                 * FAILURE:否
                */
                BT::NodeStatus tick();

                void objCB(const custom_msgs::msg::ObjectComputation::SharedPtr msg);
                private:
                bool m_autoNxt;//是否自动进入下个阶段
                custom_msgs::msg::ObjectComputation::SharedPtr m_lastMsg;//上一次目标消息
                rclcpp::Subscription<custom_msgs::msg::ObjectComputation>::SharedPtr m_subObjComputation;//目标消息订阅
        };
    }
}

#endif
