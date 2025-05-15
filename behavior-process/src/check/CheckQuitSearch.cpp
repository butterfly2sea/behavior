#include "../../include/check/CheckQuitSearch.hpp"
#include "../../include/info/Param.hpp"
#include "../../include/set/SetTraceAttackObj.hpp"
namespace zyzn{
    namespace check{
        CCheckQuitSearch::CCheckQuitSearch(const std::string & instance_name,
            const BT::NodeConfig& conf):SyncActionNode(instance_name, conf),m_autoNxt(true),m_lastMsg(nullptr),
            m_subObjComputation(nullptr){
                m_subObjComputation = info::CParam::m_glbNode->create_subscription<custom_msgs::msg::ObjectComputation>
                ("inner/information/object_computation", rclcpp::SensorDataQoS(), 
                std::bind(&CCheckQuitSearch::objCB, this, std::placeholders::_1));
        }

        PortsList CCheckQuitSearch::providedPorts(){
            return {};
        }


        NodeStatus CCheckQuitSearch::tick(){
            if(m_lastMsg && m_lastMsg->objs.size()>0){
                int64_t tn = info::CParam::m_glbNode->now().seconds();
                if(abs(tn-m_lastMsg->header.stamp.sec )<2){
                    bool fd = false;
                    //如果已经指定目标id，判定目标id同指定id一致则认为找到目标
                    if(0 != set::CSetTraceAttackObj::m_s_tgt.tgtId){
                        for(auto obj:m_lastMsg->objs){
                            if(obj.id == set::CSetTraceAttackObj::m_s_tgt.tgtId){
                                set::CSetTraceAttackObj::m_s_attckObj = obj;
                                fd = true;
                                break;
                            }
                        }
                    }else{//如没指定目标id，则认为只要收到目标位置就找到了目标
                        set::CSetTraceAttackObj::m_s_attckObj = m_lastMsg->objs[0];
                        fd = true;
                    }
                    if(m_autoNxt && fd){
                        RCLCPP_INFO(rclcpp::get_logger("CheckQuitSearch"), "found obj success");
                        return NodeStatus::SUCCESS;
                    }
                }                              
            }
            return NodeStatus::FAILURE;
        }

        void CCheckQuitSearch::objCB(const custom_msgs::msg::ObjectComputation::SharedPtr msg){
            m_lastMsg = msg;
        }

    }
}