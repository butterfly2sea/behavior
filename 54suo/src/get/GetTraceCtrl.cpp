#include "../../include/get/GetTraceCtrl.hpp"
#include "../../include/set/SetTraceAttackObj.hpp"
#include <behavior_lib/get/GetLocation.hpp>
#include <behavior_lib/info/Param.hpp>

namespace zyzn{
    namespace get{
        GetTraceCtrl::GetTraceCtrl(const std::string& name,
            const NodeConfig& conf):SyncActionNode(name, conf),m_objSub(nullptr){
            m_ctrl.x = set::CSetTraceAttackObj::objLoc().x/1e3;//使用目标的x坐标
            m_ctrl.y = set::CSetTraceAttackObj::objLoc().y/1e3;//使用目标的y坐标
            m_ctrl.z = get::CGetlocation::simpVehi().z/1e3;
            m_ctrl.yaw = get::CGetlocation::simpVehi().yaw/1e3;
            //如为固定翼则使用位置+空速控制
            if(get::CGetlocation::isFixWing()){
                m_ctrl.airspd = 0;//空速为0时飞控会使用最小空速
                m_ctrl.ordmask = EOffboardMask::LocCtrl + EOffboardMask::AirSpdCtrl;
            }else{
                m_ctrl.ordmask = EOffboardMask::LocCtrl + EOffboardMask::YawCtrl;
            }
            m_objSub = info::CParam::rosNode()->create_subscription<custom_msgs::msg::ObjectComputation>
                ("inner/information/object_computation", rclcpp::SensorDataQoS(), 
                std::bind(&GetTraceCtrl::objCB, this, std::placeholders::_1));
            int lost = 5;//默认目标丢失时长为5秒
            getInput<int>("lost", lost);
            m_lostLen = std::chrono::milliseconds(lost*1000);
            m_preTs = std::chrono::system_clock::now();
        }

        PortsList GetTraceCtrl::providedPorts(){
            return {OutputPort<custom_msgs::msg::OffboardCtrl>("target"),
                    InputPort<int>("lost")};//判断目标丢失时长,秒
        }

        NodeStatus GetTraceCtrl::tick(){
            setOutput<custom_msgs::msg::OffboardCtrl>("target",m_ctrl);
            return NodeStatus::SUCCESS;
        }

        void GetTraceCtrl::objCB(const custom_msgs::msg::ObjectComputation::SharedPtr msg){
            if((std::chrono::system_clock::now() - m_preTs) > m_lostLen){
                //超过目标丢失时长，则认为目标丢失停止更新位置信息
                return;
            }
            m_preTs = std::chrono::system_clock::now();
            float objz = 0;
            for(size_t i=0;i<msg->objs.size();++i){
                if(msg->objs[i].id == set::CSetTraceAttackObj::objLoc().id){//更新目标信息
                    m_ctrl.x = msg->objs[i].x/1e3;//使用目标位置x
                    m_ctrl.y = msg->objs[i].y/1e3;//使用目标位置y
                    objz = msg->objs[i].z/1e3;
                    break;
                }
            }

            if(get::CGetlocation::isFixWing()){
                m_ctrl.ordmask = EOffboardMask::FixLocRadCtrl;
                m_ctrl.vy = abs(get::CGetlocation::simpVehi().z/1e3-objz);//盘旋半径使用目标位置高度和飞机高度差
                if(m_ctrl.vy < 150){
                    m_ctrl.vy = 150;//盘旋半径最小150米
                }
            }
            
        }

    }
}