#ifndef ZYZN_SET_ROLE_HPP
#define ZYZN_SET_ROLE_HPP
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <custom_msgs/msg/command_request.hpp>
#include "../info/Param.hpp"

using namespace BT;
namespace zyzn{
    namespace set{
        class CSetRole : public RosTopicPubNode<custom_msgs::msg::CommandRequest>{
            public:
            enum ERole{
                Search=0,
                Detec,
                Detach
            };

            enum{
                SetRole=24//notify role
            };
            
            CSetRole(const std::string & instance_name,
            const BT::NodeConfig& conf,
            const RosNodeParams& params):RosTopicPubNode<custom_msgs::msg::CommandRequest>(instance_name,conf,params){
              
            }

            static PortsList providedPorts()
            {
                return providedBasicPorts({InputPort<int>("role")});
            }
         
            bool setMessage(custom_msgs::msg::CommandRequest& msg){
                int role = ERole::Search;
                getInput<int>("role",role);
                //zyzn::info::CParam::m_role = role;
                msg.src = zyzn::info::CParam::m_sSelfId;
                msg.type = SetRole;
                msg.dst = 0;
                msg.param0 = role;
                return true;
            }
            private:
        

        };
    }
}
#endif