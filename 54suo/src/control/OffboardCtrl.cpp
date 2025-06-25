#include "../../include/control/OffboardCtrl.hpp"
#include "../../include/info/OffboardInfo.hpp"
#include "../../include/info/Param.hpp"

namespace zyzn{
    namespace ctrl{
        COffboardCtrl::COffboardCtrl(const std::string& name,
        const NodeConfig& conf):SyncActionNode(name, conf),m_pubOffbdCtrl(nullptr){
            m_pubOffbdCtrl = info::CParam::m_glbNode->create_publisher<custom_msgs::msg::OffboardCtrl>
            ("inner/control/offboard",rclcpp::SensorDataQoS());
        }

        PortsList COffboardCtrl::providedPorts(){
            return {InputPort<custom_msgs::msg::OffboardCtrl>("ctrl")};
        }


        NodeStatus COffboardCtrl::tick(){
            static custom_msgs::msg::OffboardCtrl msg;
            if(m_pubOffbdCtrl){
                getInput<custom_msgs::msg::OffboardCtrl>("ctrl",msg);
                m_pubOffbdCtrl->publish(msg);
            }
            return NodeStatus::SUCCESS;      
        }
    }
}