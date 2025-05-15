#include "../../include/get/GetLocation.hpp"
#include "../../include/info/OffboardInfo.hpp"
#include "../../include/info/Param.hpp"
#include "../../include/control/TaskDecision.hpp"
#include "../../include/utility/Utility.hpp"
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
                OutputPort<int>("lock"),
                OutputPort<int>("flymd"),
                OutputPort<int>("locx"),
                OutputPort<int>("locy"),
                OutputPort<int>("locz"),
       
                InputPort<int>("getyaw"),
                InputPort<float>("zoffset"),
                InputPort<float>("fixed"),
                InputPort<float>("rscZ"),
                InputPort<float>("dtcZ")};
        }
        
        NodeStatus CGetlocation::onRunning(){
            if(m_simpVehi.id <= 0){
                return NodeStatus::RUNNING;
            }
            custom_msgs::msg::OffboardCtrl ctrl;//输出的offboard控制量
            ctrl.ordmask = EOffboardMask::LocCtrl+EOffboardMask::YawCtrl;//默认为位置+航向控制
            ctrl.x = m_simpVehi.x/1e3;//当前x
            ctrl.y = m_simpVehi.y/1e3;//当前y
            ctrl.yaw = m_simpVehi.yaw/1e3;//当前航向
            float zoffset = 0;
            float zfixed = m_simpVehi.z/1e3;
            ctrl.z = m_simpVehi.z/1e3;

            setOutput<int>("lock",m_simpVehi.lock);
            setOutput<int>("flymd",m_simpVehi.flymd);
            setOutput<int>("locx",m_simpVehi.x);
            setOutput<int>("locy",m_simpVehi.y);
            setOutput<int>("locz",m_simpVehi.z);
            
            if(getInput<float>("zoffset",zoffset))//zoffset有效时需要将z+offset值
                ctrl.z += zoffset;
            else if(getInput<float>("fixed",zfixed))//否则如果fixed有效，则z使用固定值
                ctrl.z = zfixed;
            else{
                ctrl.z = ctrl::CFlightmodeCtrl::takeoffZ();//m_dstAlt；//否则使用地面站设置值
            }
            algorithm::CUtility::checkZValid<float>(ctrl.z);//纠正起飞高度
            RCLCPP_INFO(rclcpp::get_logger("GetLocation"),"getlocation x: %f y:%f z:%f ",ctrl.x,ctrl.y,ctrl.z);
            
            setOutput<custom_msgs::msg::OffboardCtrl>("target",ctrl);//输出参数值更新
            //判定是否设置了home点，如未设置将当前点设置为home点
            if(info::CParam::m_sHome.z <= algorithm::CUtility::EDefVal::InvalidAlt){
                RCLCPP_INFO(rclcpp::get_logger("GetLocation"),"no home position,so set cur gps as home");
                ctrl::CTaskDecision::ins()->setHome(m_simpVehi.lat,m_simpVehi.lon,m_simpVehi.alt);
            }
            return NodeStatus::SUCCESS;
            
        }

        void CGetlocation::init(){
            if(!m_s_subSelfVehi){
                m_simpVehi.id = 0;
                m_s_subSelfVehi = info::CParam::m_glbNode->create_subscription<custom_msgs::msg::SimpleVehicle>(
                    "inner/information/simple_vehicle",rclcpp::SensorDataQoS(),
                    std::bind(&CGetlocation::simpVehiCB,this,std::placeholders::_1));
            }

        }

        void CGetlocation::simpVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg){
            m_simpVehi.id = msg->id;
            info::CParam::m_sSelfId = m_simpVehi.id;
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