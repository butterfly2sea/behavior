
#include <log/Logger.hpp>
#include "../../include/control/LockCtrl.hpp"
#include "../../include/info/Param.hpp"
#include "../../include/get/GetLocation.hpp"
#include "../../include/set/SetLine.hpp"

namespace zyzn{
    namespace ctrl{
        rclcpp::Client<custom_msgs::srv::CommandBool>::SharedPtr CLockCtrl::m_s_cltLockCtrl=nullptr;
        CLockCtrl::CLockCtrl(const std::string& name,const NodeConfig& conf):
        StatefulActionNode(name,conf),m_tickCount(0){
            if(!m_s_cltLockCtrl){
                m_s_cltLockCtrl = info::CParam::rosNode()->create_client<custom_msgs::srv::CommandBool>(
                "inner/control/lock_unlock");
            }
            getInput<int>("state",m_expState);
        }

        PortsList CLockCtrl::providedPorts()
        {
            return {InputPort<int>("state"),
            OutputPort<bool>("result")};
        }

        NodeStatus CLockCtrl::onStart(){
            if(m_s_cltLockCtrl->wait_for_service(std::chrono::milliseconds(50))){
                txtLog().info(THISMODULE "service is ready");
                return NodeStatus::RUNNING;
            }
            RCLCPP_WARN(rclcpp::get_logger("LockCtrl"),"service not ready");
            return NodeStatus::FAILURE;
        }

        NodeStatus CLockCtrl::onRunning(){
            if(m_tickCount++%set::CSetLine::TickCount == 0){
                auto req = std::make_shared<custom_msgs::srv::CommandBool::Request>();
                req->value = 1 == m_expState;
                m_s_cltLockCtrl->async_send_request(req);
                if(m_tickCount >= set::CSetLine::TickCount)
                    m_tickCount = 0;
                
            }
            if(get::CGetlocation::simpVehi().lock == m_expState){
                txtLog().info(THISMODULE "ctrl:%d success",m_expState);
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::RUNNING;
        }

        void CLockCtrl::onHalted(){

        }

        
    }
}