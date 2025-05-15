#include "../../include/control/RouteLocal.hpp"
#include "../../include/utility/Utility.hpp"
#include "../../include/info/OffboardInfo.hpp"
namespace zyzn{
    namespace ctrl{
        CRouteLocal::CRouteLocal(const std::string& name,
        const NodeConfig& conf):SyncActionNode(name, conf),
        m_curIdx(0),
        m_isCollect(false),
        m_segs(0),
        m_curSeg(0),
        m_ctrlMode(Start),
        m_node(nullptr){ 

        }

        PortsList CRouteLocal::providedPorts(){
            return {InputPort<custom_msgs::msg::OffboardCtrl>("ctrl")};
        }


        void CRouteLocal::setLine(const geometry_msgs::msg::Polygon &line){
            m_wpItems = line;
            m_curIdx = 0;
            if(m_wpItems.points.size() > 0){
                m_posPilot = m_wpItems.points[0];
                m_tgtPos = m_posPilot;
            }
        }

        NodeStatus CRouteLocal::tick(){

            /*
            msg.ordmask = EOffboardMask::LocCtrl;//位置控制
            updatePilot();
            msg.x = m_posPilot.x;
            msg.y = m_posPilot.y;
            msg.z = m_posPilot.z;
            RCLCPP_INFO(node_->get_logger(),"localroute setmessage x:%f y:%f z:%f",
            msg.x,msg.y,msg.z);
            if(Stop==m_ctrlMode)//如当前控制指令为停止，则停止发送控制消息
                return false;
            return true;
            */
            return NodeStatus::SUCCESS;
        }

        void CRouteLocal::setNode(rclcpp::Node::SharedPtr nd){
            m_node = nd;
            if(m_node){
                    m_subSimpVehi = m_node->create_subscription<custom_msgs::msg::SimpleVehicle>(
            "/inner/information/simple_vehicle", rclcpp::SensorDataQoS(), std::bind(&CRouteLocal::simpleVehicleCallback, this,
            std::placeholders::_1));

            m_srvGetDisTarget = m_node->create_service<custom_msgs::srv::GetDisTarget>( 
                "/inner/get/waypt_dis", std::bind(&CRouteLocal::getDisTargetCallback, this,
                std::placeholders::_1, std::placeholders::_2));
            m_srvCtrSwitch = m_node->create_service<custom_msgs::srv::CommandInt>("/inner/control/form_switch", std::bind(&CRouteLocal::ctrSwitchCallback, this,
                std::placeholders::_1, std::placeholders::_2));
            }
        }

        bool CRouteLocal::isNearPilot(){
            
            float dis = algorithm::CUtility::getDisFrmLoc(m_tgtPos.x,m_tgtPos.y,m_tgtPos.z,
            m_curPos.x,m_curPos.y,m_curPos.z);
            if(dis <= 0.3)
                return true;
            return false;
        }

        bool CRouteLocal::isNearMid(){
            float dis = algorithm::CUtility::getDisFrmLoc(m_posPilot.x,m_posPilot.y,m_posPilot.z,
            m_curPos.x,m_curPos.y,m_curPos.z);
            if(dis <= 0.3)
                return true;
            return false;
        }

        void CRouteLocal::updatePilot(){
            if(Pause == m_ctrlMode)
                return;
            if(isNearPilot()){
                toNextWp();
            }
            else{
                if(isNearMid()){
                
                    ++m_curSeg;
                    float pm = 1;
                    if(m_curSeg>m_segs){
                        pm = m_curSeg - m_segs;
                    }
                    m_posPilot.x += m_diffx*pm;
                    m_posPilot.y += m_diffy*pm;
                    m_posPilot.z += m_diffz*pm;
                    //toNextWp();
                }
                
            }
        }

        void CRouteLocal::toNextWp(){
            int preIdx = m_curIdx;
            ++m_curIdx;
            if(m_curIdx>=m_wpItems.points.size()){
                m_curIdx = 0;
            }
            if(m_wpItems.points.size()>1){
                float dis = algorithm::CUtility::getDisFrmLoc(m_wpItems.points[preIdx].x,m_wpItems.points[preIdx].y,m_wpItems.points[preIdx].z,
                m_wpItems.points[m_curIdx].x,m_wpItems.points[m_curIdx].y,m_wpItems.points[m_curIdx].z);
                m_segs = dis/0.5;
                if(m_segs<1)
                    m_segs = 1;
                m_curSeg = 1;
                
                m_diffx = (m_wpItems.points[m_curIdx].x - m_wpItems.points[preIdx].x)/m_segs;
                m_diffy = (m_wpItems.points[m_curIdx].y - m_wpItems.points[preIdx].y)/m_segs;
                m_diffz = (m_wpItems.points[m_curIdx].z - m_wpItems.points[preIdx].z)/m_segs;
                m_tgtPos = m_wpItems.points[m_curIdx];
            }
        }

        void CRouteLocal::simpleVehicleCallback(const custom_msgs::msg::SimpleVehicle::SharedPtr msg){
            m_curPos.x = msg->x/1e3;
            m_curPos.y = msg->y/1e3;    
            m_curPos.z = msg->z/1e3;
            RCLCPP_INFO(m_node->get_logger(),"local x:%d y:%d z:%d",
            msg->x,msg->y,msg->z);
        }

        void CRouteLocal::getDisTargetCallback(const custom_msgs::srv::GetDisTarget::Request::SharedPtr request,
                                    custom_msgs::srv::GetDisTarget::Response::SharedPtr response){

            response.get()->rslt.dis = algorithm::CUtility::getDisFrmLoc(m_curPos.x,m_curPos.y,m_curPos.z,
            m_tgtPos.x,m_tgtPos.y,m_tgtPos.z);
            response.get()->rslt.id = m_curIdx+1;
            response.get()->success = true;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get dis target");

        }


        void CRouteLocal::ctrSwitchCallback(const custom_msgs::srv::CommandInt::Request::SharedPtr request,  
                                    custom_msgs::srv::CommandInt::Response::SharedPtr response){
            m_ctrlMode = request.get()->command;
            response.get()->success = true;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ctr switch");
        }
    }
}