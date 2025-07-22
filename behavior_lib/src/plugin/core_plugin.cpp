#include <geometry_msgs/msg/point32.hpp>
#include <json/json.h>
#include <plugin/core_plugin.hpp>
#include <check/CheckArriveDst.hpp>
#include <check/CheckStartTask.hpp>
#include <check/CheckWayViaTp.hpp>
#include <check/CheckAllArriveDst.hpp>
#include <set/SetTrigger.hpp>
#include <set/SetLine.hpp>
#include <set/SetDstPt.hpp>
#include <get/GetLocation.hpp>
#include <get/GetGroupLocation.hpp>
#include <control/Joystick.hpp>
#include <control/LockCtrl.hpp>
#include <control/FlightmodeCtrl.hpp>
#include <control/OffboardCtrl.hpp>
#include <control/NavwayCtrl.h>
#include <control/TraceAttackCtrl.hpp>
#include <status/CommandStatus.hpp>
#include <log/Logger.hpp>
#include "../../include/utility/Utility.hpp"
namespace zyzn{
    namespace plugin{
        bool CorePlugin::parseJsonParam(const Json::Value &json){
            return true;
        }

        void CorePlugin::registerNode( BT::BehaviorTreeFactory &factory){

            factory.registerNodeType<check::CCheckArriveDst>("CheckArriveDst");
            factory.registerNodeType<check::CCheckStartTask>("CheckStartTask");
            factory.registerNodeType<check::CCheckWayViaTp>("CheckWayViaTp");
            factory.registerNodeType<check::CCheckAllArriveDst>("CheckAllArriveDst");
            factory.registerNodeType<set::CSetTrigger>("SetTriggers");
            factory.registerNodeType<set::CSetLine>("SetLine");
            factory.registerNodeType<set::CSetDstPt>("SetDstPt");
            factory.registerNodeType<get::CGetlocation>("GetLocation");
            factory.registerNodeType<get::CGetGroupLocation>("GetGroupLocation");
            factory.registerNodeType<ctrl::CJoystick>("Joystick");
            factory.registerNodeType<ctrl::CLockCtrl>("LockCtrl");
            factory.registerNodeType<ctrl::CFlightmodeCtrl>("FlightmodeCtrl");
            factory.registerNodeType<ctrl::COffboardCtrl>("OffboardCtrl");
            factory.registerNodeType<ctrl::CNavwayCtrl>("NavwayCtrl");
            factory.registerNodeType<ctrl::CTraceAttackCtrl>("TraceAttackCtrl");
            factory.registerNodeType<status::CCommandStatus>("CommandStatus");

        }

        bool CorePlugin::procCmd(const custom_msgs::msg::CommandRequest::SharedPtr msg,std::string & treeName){
            bool ret = true;
            treeName.clear();
            if(status::CCommandStatus::ECmd::Takeoff == msg->type){
                treeName="TakeOff-start";
                //ctrl::CFlightmodeCtrl::takeoffZ() = msg->param2/1e3;
                float z = msg->param2/1e3;
                if(abs(z)<0.1){
                    z = -1;
                }
                Json::Value alt(Json::objectValue);
                alt["value"] = z;
                info::CParam::updateParam("alt",alt);
            }else if(status::CCommandStatus::ECmd::Loiter == msg->type){
                treeName="LoitTree";
            }else if(status::CCommandStatus::ECmd::Joystick == msg->type){
                if(msg->param0){
                    if(!ctrl::CJoystick::m_s_startJoy){
                        treeName="JosystickTree";
                    }
                }
            }else if(status::CCommandStatus::ECmd::Point == msg->type){//指点飞行
                //保存位置点信息
                treeName = "GotoDst-start";
                if(0==msg->param0){//gps时将gps转换为loc
                    set::CSetDstPt::dstPt().dstPt.x = msg->param1/1e7;
                    set::CSetDstPt::dstPt().dstPt.y = msg->param2/1e7;
                    set::CSetDstPt::dstPt().dstPt.z = msg->param3/1e3-info::CParam::home().z;
                    algorithm::CUtility::gps2loc(info::CParam::home(),set::CSetDstPt::dstPt().dstPt);
                }else if(1==msg->param0){//loc
                    set::CSetDstPt::dstPt().dstPt.x = msg->param1/1e3;
                    set::CSetDstPt::dstPt().dstPt.y = msg->param2/1e3;
                    set::CSetDstPt::dstPt().dstPt.z = msg->param3/1e3;
                }else{
                    txtLog().warnning(THISMODULE "不支持的指点飞行类型%d",msg->param0);
                    treeName.clear();
                }
            }else{
                ret = false;//其它指令不处理
            }
            return ret;
        }

        bool CorePlugin::procJoyCtrl(std::string & treeName){
            treeName.clear();
            if(!ctrl::CJoystick::m_s_startJoy){
                treeName = "JosystickTree";

            }
            return true;
        }


    }
}