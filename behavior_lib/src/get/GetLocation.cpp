#include "../../include/get/GetLocation.hpp"
#include "../../include/info/OffboardInfo.hpp"
#include "../../include/info/Param.hpp"
#include "../../include/control/FlightmodeCtrl.hpp"
#include "../../include/utility/Utility.hpp"
#include <set/SetLine.hpp>
namespace zyzn{
    namespace get{

        custom_msgs::msg::SimpleVehicle CGetlocation::m_simpVehi;
        rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr CGetlocation::m_s_subSelfVehi=nullptr;
        CGetlocation::CGetlocation(const std::string& name,
        const NodeConfig& conf):StatefulActionNode(name, conf){
            init();
        }

        CGetlocation::CGetlocation():StatefulActionNode("",NodeConfig()){
            init();
        }

        PortsList CGetlocation::providedPorts(){
            return {OutputPort<custom_msgs::msg::OffboardCtrl>("target"),
            OutputPort<int>("mode"),
            InputPort<float>("zoffset"),
            InputPort<float>("fixed")};
        }
        
        NodeStatus CGetlocation::onRunning(){
            if(m_simpVehi.id <= 0){
                return NodeStatus::RUNNING;
            }
            setOutput<int>("mode",m_simpVehi.flymd);
            custom_msgs::msg::OffboardCtrl ctrl;//输出的offboard控制量
            ctrl.ordmask = EOffboardMask::LocCtrl+EOffboardMask::YawCtrl;//默认为位置+航向控制
            ctrl.x = m_simpVehi.x/1e3;//当前x
            ctrl.y = m_simpVehi.y/1e3;//当前y
            ctrl.yaw = m_simpVehi.yaw/1e3;//当前航向
            float zoffset = 0;
            float zfixed = m_simpVehi.z/1e3;
            ctrl.z = m_simpVehi.z/1e3;
            
            if(getInput<float>("zoffset",zoffset))//zoffset有效时需要将z+offset值
                ctrl.z += zoffset;
            else if(getInput<float>("fixed",zfixed))//否则如果fixed有效，则z使用固定值
                ctrl.z = zfixed;
            else{
                ctrl.z = ctrl::CFlightmodeCtrl::takeoffZ();//m_dstAlt；//否则使用地面站设置值
            }
            //如为垂起-固定翼则改变控制mask为行路点+空速
            if((set::CSetLine::FixWing==m_simpVehi.type) || (set::CSetLine::VtolFix == m_simpVehi.type)){
                ctrl.airspd = 0;//空速为0时飞控会使用最小空速
                ctrl.ordmask = EOffboardMask::LocCtrl + EOffboardMask::AirSpdCtrl;
            }
            algorithm::CUtility::checkZValid<float>(ctrl.z);//纠正起飞高度
  
            
            setOutput<custom_msgs::msg::OffboardCtrl>("target",ctrl);//输出参数值更新
            
            return NodeStatus::SUCCESS;
        }

        bool CGetlocation::isFixWing(){
            if((set::CSetLine::FixWing==m_simpVehi.type) || (set::CSetLine::VtolFix == m_simpVehi.type)){
                return true;
            }
            return false;
        }

        void CGetlocation::init(){
            if(!m_s_subSelfVehi){
                m_simpVehi.id = 0;
                m_s_subSelfVehi = info::CParam::rosNode()->create_subscription<custom_msgs::msg::SimpleVehicle>(
                    "inner/information/simple_vehicle",rclcpp::SensorDataQoS(),
                    std::bind(&CGetlocation::simpVehiCB,this,std::placeholders::_1));
            }

        }

        void CGetlocation::simpVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg){
            m_simpVehi.id = msg->id;
            info::CParam::vehiId() = m_simpVehi.id;
            m_simpVehi.airspd = msg->airspd;
            m_simpVehi.x = msg->x;
            m_simpVehi.y = msg->y;
            m_simpVehi.z = msg->z;
            m_simpVehi.flymd = msg->flymd;
            m_simpVehi.lock = msg->lock;
            m_simpVehi.lat = msg->lat;
            m_simpVehi.lon = msg->lon;
            m_simpVehi.alt = msg->alt;
            m_simpVehi.yaw = msg->yaw;
            m_simpVehi.type = msg->type;
        }
    }
}