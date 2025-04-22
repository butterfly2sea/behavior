
#include <log/Logger.hpp>
#include "../../include/control/NavwayCtrl.h"
#include "../../include/info/Param.hpp"

namespace zyzn{
    namespace ctrl{
        rclcpp::Client<custom_msgs::srv::CommandInt>::SharedPtr CNavwayCtrl::m_s_navCtrl=nullptr;
        CNavwayCtrl::CNavwayCtrl(const std::string & instance_name,
        const BT::NodeConfig& conf):StatefulActionNode(instance_name,conf){
            if(!m_s_navCtrl){
                m_s_navCtrl = info::CParam::rosNode()->create_client<custom_msgs::srv::CommandInt>(
            "inner/control/form_switch");
            }
        }

        PortsList CNavwayCtrl::providedPorts(){
            return {InputPort<int>("frame"),
            InputPort<int>("command")};
        }


        NodeStatus CNavwayCtrl::onStart(){
            if(m_s_navCtrl->wait_for_service(std::chrono::milliseconds(50))){
                txtLog().info(THISMODULE "service is ready");
                return NodeStatus::RUNNING;
            }
            txtLog().warnning(THISMODULE "service not ready");
            return NodeStatus::FAILURE;
        }

        NodeStatus CNavwayCtrl::onRunning(){
            auto request = std::make_shared<custom_msgs::srv::CommandInt::Request>();
            Expected<uint8_t>  frame = getInput<int>("frame");
            Expected<uint16_t> command = getInput<int>("command");
            if(frame){
                request->frame = frame.value();
            }
            else
                request->frame = 1;//没有参数则使用1（节点自己进行offboard控制）
            if(command){
                request->command = command.value();
            }
            else
                request->command =  0;//没有参数则使用0（开始编队飞行）
            
            m_s_navCtrl->async_send_request(request);
            txtLog().info(THISMODULE "navctrl frame:%d command:%d",request->frame,request->command);
            return NodeStatus::SUCCESS;
        }

        void CNavwayCtrl::onHalted(){
            
        }

    }
}