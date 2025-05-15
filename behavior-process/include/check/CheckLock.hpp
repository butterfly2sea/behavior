#ifndef ZYZN_CHECK_LOCK_HPP
#define ZYZN_CHECK_LOCK_HPP
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
using namespace BT;
namespace zyzn{
    namespace check{
        /**
         * @brief 检测锁状态是否和期望状态一致
         * @details 定时检测简单飞控消息中的上锁状态是否和期望状态一致
         * @author
        */
        class CCheckLock : public RosTopicSubNode<custom_msgs::msg::SimpleVehicle>{
            public:
            /**
             * 上锁状态
            */
            enum ELockState{
                Lock=0,      ///<!上锁>
                Unlock,      ///<!解锁>

            };

            CCheckLock(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params):RosTopicSubNode<custom_msgs::msg::SimpleVehicle>(name,conf,params){

            }

            /**
             * @brief 输入输出参数申明
             * @details state：期望上锁状态输入参数
             * @return 输入输出参数列表
            */
            static PortsList providedPorts()
            {
                return providedBasicPorts({InputPort<int>("state")});
            }
            
            /**
             * @brief  节点定时回调
             * @details
             * @return 上锁状态是否和期望值一致
             * SUCCESS:一致
             * FAILURE:不一致
            */
            NodeStatus onTick(const  custom_msgs::msg::SimpleVehicle::SharedPtr& last_msg){
                //RCLCPP_INFO(node_->get_logger(),"check lock ontick");
                getInput<int>("state",m_expState);
                if(last_msg){
                    RCLCPP_INFO(node_->get_logger(), "lock check exp:%d rcv:%d", m_expState,last_msg->lock);
                }
                if(last_msg && last_msg->lock==m_expState){
                    RCLCPP_INFO(node_->get_logger(), "lock ctrl success md:%d", m_expState);
                    return NodeStatus::SUCCESS;
                }
                else
                    return NodeStatus::FAILURE;
                
            }
            private:
            int m_expState;//期望上锁状态用于保存输入参数值
            
        };
    }
}

#endif