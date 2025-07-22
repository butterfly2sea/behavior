#include <stdexcept>
#include <action_libs/trace_attack_plugin.hpp>
#include <set/SetTraceAttackObj.hpp>
#include <check/CheckQuitSearch.hpp>
#include <behavior_lib/control/TraceAttackCtrl.hpp>
#include <behavior_lib/status/CommandStatus.hpp>
#include <behavior_lib/plugin/interface.hpp>
#include <control/FuelUp.hpp>
#include <control/CameraCtrl.hpp>
#include <status/TraceStatus.hpp>
#include <get/GetTraceCtrl.hpp>
#include <get/GetTraceInfo.hpp>
#include <log/Logger.hpp>

namespace zyzn{
    namespace plugin{

        void TraceAttackPlugin::registerNode(BT::BehaviorTreeFactory &factory){

            factory.registerNodeType<set::CSetTraceAttackObj>("SetTraceAttackObj");
            factory.registerNodeType<check::CCheckQuitSearch>("CheckQuitSearch");
            factory.registerNodeType<ctrl::CCameraCtrl>("CameraCtrl");
            factory.registerNodeType<ctrl::CFuelUp>("FuelUp");
            factory.registerNodeType<status::CTraceStatus>("TraceStatus");
            factory.registerNodeType<get::GetTraceCtrl>("GetTraceCtrl");
            factory.registerNodeType<get::CTraceInfo>("TraceInfo");
        }
            
        bool TraceAttackPlugin::procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,
        std::string & treeName){
            
            status::CCommandStatus::cmdRspMsg().src = status::CCommandStatus::ECtrlType::TraceAttack;
            status::CCommandStatus::cmdRspMsg().type = msg.get()->type;

            if((ctrl::CTraceAttackCtrl::UpdateMixObj == msg->type) || 
            (ctrl::CTraceAttackCtrl::UpdateGrdObj == msg->type)){
                status::CTraceStatus::mixObj().objLat = msg->objs.x;
                status::CTraceStatus::mixObj().objLon = msg->objs.y;
                status::CTraceStatus::mixObj().objAlt = msg->objs.z;
                return true;
            }
            txtLog().info(THISMODULE "procTraceAttack: type:%d",msg->type);
            if(ctrl::CTraceAttackCtrl::Abort == msg->type){
                treeName = "LoitTree";
            }else  if(msg->type>=ctrl::CTraceAttackCtrl::EleMagAccLd && msg->type<=ctrl::CTraceAttackCtrl::EleMagVerLd){//电磁干扰攻击只降落
                treeName = "LandTree";
            }else if(POD_GPS_LOCK == msg->type){
                status::CTraceStatus::mixObj().objLat = msg->objs.x;
                status::CTraceStatus::mixObj().objLon = msg->objs.y;
                status::CTraceStatus::mixObj().objAlt = msg->objs.z;
                treeName = "PodGpsLock-start-1";//吊舱gps锁定开始
                
            }else if(CANCEL_POD_GPS_LOCK == msg->type){
                treeName = "PodGpsLock-stop-1";//吊舱gps锁定结束
            }else{//跟踪打击
                set::CSetTraceAttackObj::tgtInfo().tgtId = 0;
                set::CSetTraceAttackObj::objLoc() = msg.get()->objs;
                status::CTraceStatus::gps2loc(msg->objs.x,msg->objs.y,msg->objs.z,set::CSetTraceAttackObj::objLoc().x,
                set::CSetTraceAttackObj::objLoc().y,set::CSetTraceAttackObj::objLoc().z);
                ctrl::CTraceAttackCtrl::m_s_traceAttackType= msg.get()->type;
                treeName = "Attack-start";
            }

            return true;
        }
        

        std::shared_ptr<BasePlugin> create_plugin(){
            return std::make_shared<TraceAttackPlugin>();
        }
    }
}