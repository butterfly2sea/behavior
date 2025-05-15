#include <action_libs/core_plugin.hpp>
#include <check/CheckArriveDst.hpp>
#include <check/CheckStartTask.hpp>
#include <set/SetTrigger.hpp>
#include <set/SetLine.hpp>
#include <get/GetLocation.hpp>
#include <control/Joystick.hpp>
#include <control/LockCtrl.hpp>
#include <control/FlightmodeCtrl.hpp>
#include <control/OffboardCtrl.hpp>
#include <control/NavwayCtrl.h>
#include <control/TraceAttackCtrl.hpp>
#include <status/CommandStatus.hpp>
namespace zyzn{
    namespace plugin{
        bool CorePlugin::parseJsonParam(const Json::Value &json){
            return true;
        }

        void CorePlugin::registerNode( BT::BehaviorTreeFactory &factory){

            factory.registerNodeType<check::CCheckArriveDst>("CheckArriveDst");
            factory.registerNodeType<check::CCheckStartTask>("CheckStartTask");
            factory.registerNodeType<set::CSetTrigger>("SetTriggers");
            factory.registerNodeType<set::CSetLine>("SetLine");
            factory.registerNodeType<get::CGetlocation>("GetLocation");
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
                treeName="TakeoffTree";
            }else if(status::CCommandStatus::ECmd::Loiter == msg->type){
                treeName="LoitTree";
            }else if(status::CCommandStatus::ECmd::Joystick == msg->type){
                if(msg->param0){
                    if(!ctrl::CJoystick::m_s_startJoy){
                        treeName="JosystickTree";
                    }
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