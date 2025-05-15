#ifndef ZYZN_CTRL_FUELUP_HPP
#define ZYZN_CTRL_FUELUP_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
using namespace BT;
namespace zyzn{
    namespace ctrl{
        class CFuelUp : public BT::SyncActionNode{
            public:
            enum ECtrl{
                Start=0,
                Stop=3
            };
            CFuelUp(const std::string & instance_name,
            const BT::NodeConfig& conf);

            static PortsList providedPorts();

            NodeStatus tick();
            private:
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubAlt;//加油高度
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_pubFuelId;//加油机id
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_pubCtrl;//加油控制0:开始,3:结束
            float m_fuelZ;//加油高度
            uint8_t m_fuelId;//加油机id
        };
    }
}


#endif