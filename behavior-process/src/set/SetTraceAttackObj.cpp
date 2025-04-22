#include "../../include/set/SetTraceAttackObj.hpp"
#include <control/TraceAttackCtrl.hpp>
#include <info/Param.hpp>
#include <plugin/base_plugin.hpp>
namespace zyzn{
    namespace set{
        custom_msgs::msg::ObjectLocation CSetTraceAttackObj::m_s_attckObj;
        CSetTraceAttackObj::STgtInfo CSetTraceAttackObj::m_s_tgt;
        CSetTraceAttackObj::CSetTraceAttackObj(const std::string &instance_name,
        const BT::NodeConfig &conf) : SyncActionNode(instance_name, conf),m_pubTraceAttackObj(nullptr){
            m_pubTraceAttackObj = info::CParam::rosNode()->create_publisher<custom_msgs::msg::ObjectLocation>(
             "inner/set/trace_attack_object", 10);
            std::string name = "tgtId";
            getInput<std::string>("tgtIdParam", name);
            set::CSetTraceAttackObj::m_s_tgt.tgtId = plugin::BasePlugin::getValue(info::CParam::getParam(name));

            name = "srcId";
            getInput<std::string>("srcIdParam", name);
            set::CSetTraceAttackObj::m_s_tgt.srcId = plugin::BasePlugin::getValue(info::CParam::getParam(name));
        }

        PortsList CSetTraceAttackObj::providedPorts(){
            return {InputPort<bool>("filter"),
                    InputPort<std::string>("tgtIdParam"),//json任务中目标参数名
                    InputPort<std::string>("srcIdParam"),//json任务中目标来源的飞机id
            };
        }

        NodeStatus CSetTraceAttackObj::tick(){
            bool filter = false;
            getInput<bool>("filter", filter);
            if(!filter || (filter && m_s_tgt.tgtId == ctrl::CTraceAttackCtrl::UpdateGrdObj)){
                m_pubTraceAttackObj->publish(m_s_attckObj);
                return NodeStatus::SUCCESS;
            }
            return NodeStatus::FAILURE;
        }
    }
}