#include "../../include/control/FuelUp.hpp"
#include <behavior_lib/info/Param.hpp>
#include <behavior_lib/plugin/base_plugin.hpp>
#
namespace zyzn{
    namespace ctrl{
        CFuelUp::CFuelUp(const std::string & instance_name,
            const BT::NodeConfig& conf):SyncActionNode(instance_name,
             conf),m_pubAlt(nullptr),m_pubFuelId(nullptr),
            m_pubCtrl(nullptr){
                m_pubAlt = info::CParam::rosNode()->create_publisher<std_msgs::msg::Float32>
                ("inner/set/fuel_z",rclcpp::SensorDataQoS());
                m_pubFuelId = info::CParam::rosNode()->create_publisher<std_msgs::msg::UInt8>
                ("inner/set/fuel_id",rclcpp::SensorDataQoS());
                m_pubCtrl = info::CParam::rosNode()->create_publisher<std_msgs::msg::UInt8>
                ("inner/control/fuel_switch",rclcpp::SensorDataQoS());
                std::string name = "z";
                getInput("zParam",name);
                m_fuelZ = plugin::BasePlugin::getValue(info::CParam::getParam(name));
                name = "fuelId";
                getInput("fuelId",name);
                m_fuelId = plugin::BasePlugin::getValue(info::CParam::getParam(name));

            }
    

        PortsList CFuelUp::providedPorts(){
            return {InputPort<int>("ctrl"),//控制类型0:开始,3:停止
                    InputPort<std::string>("zParam"),//加油飞行高度
                    InputPort<std::string>("fuIdParam")//加油机id
                    };
        }

        NodeStatus CFuelUp::tick(){
            //开始任务才需要发送高度及id
            int ctrl=ECtrl::Stop;
            getInput<int>("ctrl",ctrl);
            if(Start == ctrl){
                
                if(m_pubAlt){
                    std_msgs::msg::Float32 zMsg;
                    zMsg.data = m_fuelZ;
                    m_pubAlt->publish(zMsg);
                }
                if(m_pubFuelId){
                    std_msgs::msg::UInt8 idMsg;
                    idMsg.data = m_fuelId;
                    m_pubFuelId->publish(idMsg);
                }
            }
            if(m_pubCtrl){
                std_msgs::msg::UInt8 ctrlMsg;
                ctrlMsg.data = ctrl;
                m_pubCtrl->publish(ctrlMsg);
            }
            return NodeStatus::SUCCESS;
        }
        
    }
}