#include <log/Logger.hpp>
#include "../../include/check/CheckWayViaTp.hpp"
#include "../../include/info/OffboardInfo.hpp"
#include "../../include/set/SetLine.hpp"
#include "../../include/info/Param.hpp"
namespace zyzn{
    namespace check{
        custom_msgs::msg::DisTarget CCheckWayViaTp::m_s_curNavInfo;
        int CCheckWayViaTp::m_s_preNavId = 0;
        CCheckWayViaTp::CCheckWayViaTp(const std::string& name,
        const NodeConfig& conf):
        SyncActionNode(name, conf),m_accIdx(0),m_preWayIdx(0),m_subWayPt(nullptr){
            m_s_curNavInfo.id = MAX_NAV_ID;//初始化
            m_subWayPt = info::CParam::rosNode()->create_subscription<custom_msgs::msg::DisTarget>(
            "inner/information/way_point",rclcpp::SensorDataQoS(),
            std::bind(&CCheckWayViaTp::wayPtDisCB,this,std::placeholders::_1));
        }

        PortsList CCheckWayViaTp::providedPorts(){
            return {InputPort<float>("arriveDis")};
        }


        NodeStatus CCheckWayViaTp::tick(){
            if(MAX_NAV_ID == m_s_curNavInfo.id){//没有目标航点
                return NodeStatus::FAILURE;
            }
            float arvDis = 1;
            if(!getInput<float>("arriveDis",arvDis)){
                arvDis = set::CSetLine::arvDis();
            }
            

            if(m_preWayIdx != m_s_curNavInfo.id){

                m_preWayIdx = m_s_curNavInfo.id;
                m_accIdx++;
            }
            if(m_s_curNavInfo.dis <= arvDis){
                size_t len = set::CSetLine::loops()*set::CSetLine::wayPts().points.size()-1;
                if(len >=0 && (m_accIdx >= len || m_preWayIdx >= len)){
                    txtLog().info(THISMODULE  "navway all finish id:%d accIdx:%d",m_s_curNavInfo.id ,m_accIdx);
                    return NodeStatus::SUCCESS;
                } 
            }
                                
            return NodeStatus::FAILURE;
        }

        void CCheckWayViaTp::wayPtDisCB(const custom_msgs::msg::DisTarget::SharedPtr msg){
            m_s_curNavInfo.id = msg->id;
            m_s_curNavInfo.dis = msg->dis/1e3;
        }
    }
}