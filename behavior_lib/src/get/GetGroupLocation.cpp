#include <log/Logger.hpp>
#include "../../include/get/GetGroupLocation.hpp"
#include "../../include/set/SetLine.hpp"
#include "../../include/info/Param.hpp"

namespace zyzn{
    namespace get{
        CGetGroupLocation::vehis_t CGetGroupLocation::m_vehis;

        CGetGroupLocation::CGetGroupLocation(const std::string & name,const BT::NodeConfig & config):SyncActionNode(name,config),
            m_subSelfVehi(nullptr),
            m_subOutVehi(nullptr){
                m_subOutVehi = zyzn::info::CParam::rosNode()->create_subscription<custom_msgs::msg::SimpleVehicle>(
                "outer/information/simple_vehicle",rclcpp::SensorDataQoS(),
                std::bind(&CGetGroupLocation::outerVehiCB,this,std::placeholders::_1));

                m_subSelfVehi = zyzn::info::CParam::rosNode()->create_subscription<custom_msgs::msg::SimpleVehicle>(
                "inner/information/simple_vehicle",rclcpp::SensorDataQoS(),
                std::bind(&CGetGroupLocation::innerVehiCB,this,std::placeholders::_1));

            }

            bool CGetGroupLocation::hasAllVehis(){
                if(m_vehis.size() < set::CSetLine::ids().size()){
                    return false;
                }
                for(size_t i=0;i<set::CSetLine::ids().size();++i){
                    id_t id = set::CSetLine::ids()[i];
                    if(m_vehis.find(id) == m_vehis.end())
                        return false;
                }
                return true;
            }

            NodeStatus CGetGroupLocation::tick(){
                txtLog().info(THISMODULE "tick");
                return NodeStatus::SUCCESS;
            }

            void CGetGroupLocation::innerVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg){
                procMsg(msg);
            }

            void CGetGroupLocation::outerVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg){
                procMsg(msg);
            }

            void CGetGroupLocation::procMsg(const custom_msgs::msg::SimpleVehicle::SharedPtr msg){
                m_vehis[msg->id].x = msg->x;
                m_vehis[msg->id].y = msg->y;
                m_vehis[msg->id].z = msg->z;
                m_vehis[msg->id].airspd = msg->airspd;
                m_vehis[msg->id].type = msg->type;
            }

            bool CGetGroupLocation::getSelfLoc(loc_t & loc){
                return getLoc(info::CParam::vehiId(),loc);
            }

            bool CGetGroupLocation::getLoc(id_t id,loc_t & loc){
                if(m_vehis.find(id) != m_vehis.end()){
                    loc.x = m_vehis[id].x/1e3;
                    loc.y = m_vehis[id].y/1e3;
                    loc.z = m_vehis[id].z/1e3;
                    return true;
                }
                return false;
            }

            bool CGetGroupLocation::getSelfAtti(loc_t & atti){
                return getAtti(info::CParam::vehiId(),atti);
            }

            bool CGetGroupLocation::getAtti(id_t id,loc_t & atti){
                if(m_vehis.find(id) != m_vehis.end()){
                    atti.x = m_vehis[id].pitch/1e3;
                    atti.y = m_vehis[id].roll/1e3;
                    atti.z = m_vehis[id].yaw/1e3;
                    return true;
                }
                return false;
            }

            bool CGetGroupLocation::getMinZ(id_t & id,z_t &z){
                z = -9999;
                id = 0;
             
                for_each(m_vehis.begin(),m_vehis.end(),[&z,&id](const std::pair<id_t,custom_msgs::msg::SimpleVehicle> p){
                    if(p.second.z/1e3 > z){
                        z = p.second.z/1e3;
                        id = p.first;
                    }
                });
                return 0!=id;
            }

            const custom_msgs::msg::SimpleVehicle & CGetGroupLocation::getVehi(id_t id,bool & val){
                val = true;
                if(m_vehis.find(id) == m_vehis.end()){
                    val = false;
                    return m_vehis[0];
                }
                return m_vehis[id];
            }

            bool CGetGroupLocation::isSelfFix(){
                return (set::CSetLine::EVehiType::FixWing == m_vehis[info::CParam::vehiId()].type) || 
                (set::CSetLine::EVehiType::VtolFix == m_vehis[info::CParam::vehiId()].type); 
            }

            float CGetGroupLocation::selfYaw(){
                return m_vehis[info::CParam::vehiId()].yaw/1e3;
            }
    }
}