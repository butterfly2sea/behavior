#ifndef ZYZN_GET_OUTLOCATION_HPP
#define ZYZN_GET_OUTLOCATION_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include "../info/OffboardInfo.hpp"
#include "../info/Param.hpp"
using namespace BT;
namespace zyzn{
    namespace get{
        class CGetOutlocation : public BT::SyncActionNode{
            public:

            static PortsList providedPorts()
            {
                return {OutputPort<int>("olock"),OutputPort<int>("oflymd"),OutputPort<float>("ox"),
                OutputPort<float>("oy"),
                OutputPort<float>("oz")};
            }
            CGetOutlocation(const std::string& name, const NodeConfig& config):SyncActionNode(name,config){
                std::string tpName = "outer/information/simple_vehicle";
                m_subOutVehi = zyzn::info::CParam::m_glbNode->create_subscription<custom_msgs::msg::SimpleVehicle>(
                tpName,rclcpp::SensorDataQoS(),
                std::bind(&CGetOutlocation::outSimpleVehiCB,this,std::placeholders::_1));
            }

            static void regist(BehaviorTreeFactory &factory){
                
                factory.registerNodeType<CGetOutlocation>("GetOutlocation");
            }
            
            NodeStatus tick(){
                return NodeStatus::SUCCESS;
            }

            void outSimpleVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg){

                if(msg){
                    uint8_t expId = 0;
                    uint8_t oId = msg->id%zyzn::info::CParam::NumGrp;
                    if(0 == oId)
                        oId = zyzn::info::CParam::Throw;
                    switch (zyzn::info::CParam::m_sSelfId%zyzn::info::CParam::NumGrp)
                    {
                    
                    case zyzn::info::CParam::Search1:
                        expId = zyzn::info::CParam::Search2;
                        break;
                    case zyzn::info::CParam::Search2:
                        expId = zyzn::info::CParam::Search1;
                        break;
                    case zyzn::info::CParam::Detect:
                        expId = zyzn::info::CParam::Throw;
                        break;
                    case 0://trow
                        expId = zyzn::info::CParam::Detect;
                        break;
                    default:
                        break;
                    }
                    if(expId = oId){
                        
                        setOutput<float>("ox",msg->x/1e3);
                        setOutput<float>("oy",msg->y/1e3);
                        setOutput<float>("oz",msg->z/1e3); 
                        setOutput<int>("olock",msg->lock);
                        setOutput<int>("oflymd",msg->flymd);
                                            
                    }
                }
            }
            private:
            rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr m_subOutVehi;
            
            
        };
    }
}

#endif
