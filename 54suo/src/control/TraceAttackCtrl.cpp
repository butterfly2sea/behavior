#include "../../include/control/TraceAttackCtrl.hpp"
#include "../../include/info/Param.hpp"

namespace zyzn{
    namespace ctrl{
        int CTraceAttackCtrl::m_s_traceAttackType = 0;
        rclcpp::Client<custom_msgs::srv::CommandInt>::SharedPtr CTraceAttackCtrl::m_s_traceAttackCtrl=nullptr;
        CTraceAttackCtrl::CTraceAttackCtrl(const std::string & instance_name,
        const BT::NodeConfig& conf):StatefulActionNode(instance_name,conf){
            if(!m_s_traceAttackCtrl){
                m_s_traceAttackCtrl = info::CParam::m_glbNode->create_client<custom_msgs::srv::CommandInt>(
                "inner/control/guidance_switch");
            }
        }

       
        PortsList CTraceAttackCtrl::providedPorts() {
            return {InputPort<int>("frame"),
            InputPort<int>("command"),
            InputPort<int>("current")};
        }

        NodeStatus CTraceAttackCtrl::onStart(){
            if(m_s_traceAttackCtrl->wait_for_service(std::chrono::milliseconds(50))){
                RCLCPP_INFO(rclcpp::get_logger("traceAttack"),"service is ready");
                return NodeStatus::RUNNING;
            }
            RCLCPP_WARN(rclcpp::get_logger("traceAttack"),"service not ready");
            return NodeStatus::FAILURE;
        }

        NodeStatus CTraceAttackCtrl::onRunning(){
            auto request = std::make_shared<custom_msgs::srv::CommandInt::Request>();

            Expected<uint8_t>  frame = getInput<int>("frame");
            Expected<uint16_t> command = getInput<int>("command");
            Expected<uint16_t> current = getInput<int>("current");
            //优先从参数获取frame值，否则为1（节点自己进行offboard控制）
            if(frame){
                request->frame = frame.value();
            }
            else
                request->frame = 1;
            //优先从参数获取command值，否则为0（开始执行跟踪打击）
            if(command){
                request->command = command.value();
            }
            else
                request->command =  0;
            //优先从参数获取current值，否则地面站指定
            if(current){
                request->current = current.value();
            }
            else   
                request->current = (OTrace==m_s_traceAttackType )?ITrace:IAttack; 

            m_s_traceAttackCtrl->async_send_request(request);
            RCLCPP_INFO(rclcpp::get_logger("traceAttack"),"traceattack ctrl frame:%d command:%d current:%d",request->frame,request->command,
            request->current);
            return NodeStatus::SUCCESS;
        }

        void CTraceAttackCtrl::onHalted(){

        }


       
    }
}