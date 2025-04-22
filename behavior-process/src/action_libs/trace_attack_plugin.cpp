#include <stdexcept>
#include <trace_attack_plugin.hpp>
#include <set/SetTraceAttackObj.hpp>
#include <check/CheckQuitSearch.hpp>
#include <control/TraceAttackCtrl.hpp>
#include <status/CommandStatus.hpp>
#include <plugin/interface.hpp>
#include <control/TopicTrans.hpp>
namespace zyzn{
    namespace plugin{

        void TraceAttackPlugin::registerNode(BT::BehaviorTreeFactory &factory){

            factory.registerNodeType<set::CSetTraceAttackObj>("SetTraceAttackObj");
            factory.registerNodeType<check::CCheckQuitSearch>("CheckQuitSearch");
            factory.registerNodeType<ctrl::TopicTrans>("TopicTrans");
        }
            
        bool TraceAttackPlugin::procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,
        std::string & treeName){
           
            set::CSetTraceAttackObj::m_s_tgt.tgtId = 0;
            set::CSetTraceAttackObj::m_s_attckObj = msg.get()->objs;
            ctrl::CTraceAttackCtrl::m_s_traceAttackType = msg.get()->type;
            if(ctrl::CTraceAttackCtrl::UpdateGrdObj == msg->type || ctrl::CTraceAttackCtrl::UpdateMixObj == msg->type){
                return true;
            }
            status::CCommandStatus::cmdRspMsg().src = status::CCommandStatus::ECtrlType::TraceAttack;
            status::CCommandStatus::cmdRspMsg().type = msg.get()->type;
            if(ctrl::CTraceAttackCtrl::EleMagAccLd <= msg->type && msg->type <= ctrl::CTraceAttackCtrl::EleMagVerLd){//电磁干扰攻击只降落
                treeName = "LoitTree";
            }
            else if(ctrl::CTraceAttackCtrl::Abort == msg->type){
                treeName = "LoitTree";
            }
            else if(ctrl::CTraceAttackCtrl::OTrace == msg->type){
                treeName = "Trace-start";
            }
            else
                treeName = "Attack-start";

            return true;
        }

        std::shared_ptr<BasePlugin> create_plugin(){
            return std::make_shared<TraceAttackPlugin>();
        }
    }
}