#include "../../include/control/FlightmodeCtrl.hpp"
#include "../../include/get/GetLocation.hpp"
#include "../../include/info/Param.hpp"
#include "../../include/set/SetLine.hpp"
namespace zyzn{
    namespace ctrl{
        float CFlightmodeCtrl::m_s_takeoffZ = -1;
        rclcpp::Client<custom_msgs::srv::CommandLong>::SharedPtr CFlightmodeCtrl::m_s_cltFlymd=nullptr;
        CFlightmodeCtrl::CFlightmodeCtrl(const std::string & name,
        const BT::NodeConfig& conf):StatefulActionNode(name,conf){
            init();
            getInput<int>("mode",m_expMode);

        }
        CFlightmodeCtrl::CFlightmodeCtrl():StatefulActionNode("",BT::NodeConfig()){
            init();
        }

        PortsList CFlightmodeCtrl::providedPorts(){
            return {InputPort<int>("mode"),
                InputPort<float>("param7")};
        }

        NodeStatus CFlightmodeCtrl::onStart(){
            if(m_s_cltFlymd->wait_for_service(std::chrono::milliseconds(50))){
                RCLCPP_INFO(rclcpp::get_logger("FlymdCtrl"),"service is ready");
                return NodeStatus::RUNNING;
            }
            RCLCPP_WARN(rclcpp::get_logger("FlymdCtrl"),"service not ready");
            return NodeStatus::FAILURE;
        }

        NodeStatus CFlightmodeCtrl::onRunning(){
            if(m_tickCount++%set::CSetLine::TickCount == 0){
                auto req = std::make_shared<custom_msgs::srv::CommandLong::Request>();
                 Expected<float> param7 = getInput<float>("param7");
                if(param7){//此参数用于起飞模式时，起飞高度值
                    req->param7 = param7.value();
                }else{
                    req->param7 = m_s_takeoffZ;//起飞高度由地面站设置，保存在此处
                }
                if(Unknown != m_expMode){
                    req->command = m_expMode;
                }
              
                m_s_cltFlymd->async_send_request(req);
                if(m_tickCount >= set::CSetLine::TickCount-1)
                    m_tickCount = 0;
                
            }
            if(get::CGetlocation::simpVehi().flymd == m_expMode){
                RCLCPP_INFO(rclcpp::get_logger("FlymdCtrl"),"ctrl:%d success",m_expMode);
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::RUNNING;
        }

        void CFlightmodeCtrl::onHalted(){

        }

        void CFlightmodeCtrl::updateMode(int md){
            m_expMode = md;
            resetTick();
            if(onStart()==NodeStatus::FAILURE){
                RCLCPP_WARN(rclcpp::get_logger("FlymdCtrl"),"service not ready");
                return;
            }
            onRunning();
        }

        void CFlightmodeCtrl::init(){
            m_expMode = Unknown;
            m_tickCount = 0;
            if(!m_s_cltFlymd){
                m_s_cltFlymd = info::CParam::m_glbNode->create_client<custom_msgs::srv::CommandLong>(
                "inner/control/set_flymode");
            }
        }

        
    }
}