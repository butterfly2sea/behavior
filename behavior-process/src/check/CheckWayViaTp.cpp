#include "../../include/check/CheckWayViaTp.hpp"
#include "../../include/info/OffboardInfo.hpp"
#include "../../include/set/SetLine.hpp"
#include "../../include/info/Param.hpp"
namespace zyzn{
    namespace check{
        CCheckWayViaTp::CCheckWayViaTp(const std::string& name,
        const NodeConfig& conf):
        SyncActionNode(name, conf),m_accIdx(0),m_preWayIdx(0),m_subWayPt(nullptr){
            info::CParam::m_sCurNavInfo.id = MAX_NAV_ID;//初始化
            m_subWayPt = info::CParam::m_glbNode->create_subscription<custom_msgs::msg::DisTarget>(
            "inner/information/way_point",rclcpp::SensorDataQoS(),
            std::bind(&CCheckWayViaTp::wayPtDisCB,this,std::placeholders::_1));
        }

        PortsList CCheckWayViaTp::providedPorts(){
            return {InputPort<float>("arriveDis")};
        }


        NodeStatus CCheckWayViaTp::tick(){
            if(MAX_NAV_ID == info::CParam::m_sCurNavInfo.id){//没有目标航点
                return NodeStatus::FAILURE;
            }
            float arvDis = 1;
            if(!getInput<float>("arriveDis",arvDis)){
                arvDis = set::CSetLine::arvDis();
            }
            

            if(m_preWayIdx != info::CParam::m_sCurNavInfo.id){

                m_preWayIdx = info::CParam::m_sCurNavInfo.id;
                m_accIdx++;
            }
            if(info::CParam::m_sCurNavInfo.dis <= arvDis){
                size_t len = set::CSetLine::loops()*set::CSetLine::wayPts().points.size()-1;
                if(len >=0 && (m_accIdx >= len || m_preWayIdx >= len)){
                    RCLCPP_INFO(rclcpp::get_logger("CheckWayViaTp"), "navway all finish id:%d accIdx:%d",info::CParam::m_sCurNavInfo.id ,m_accIdx);
                    return NodeStatus::SUCCESS;
                } 
            }
                                
            return NodeStatus::FAILURE;
        }

        void CCheckWayViaTp::wayPtDisCB(const custom_msgs::msg::DisTarget::SharedPtr msg){
            info::CParam::m_sCurNavInfo.id = msg->id;
            info::CParam::m_sCurNavInfo.dis = msg->dis/1e3;
        }
    }
}