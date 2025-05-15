#include "../../include/set/SetTrigger.hpp"

namespace zyzn{
    namespace set{
        CSetTrigger::CSetTrigger(const std::string& name, const NodeConfig& config):SyncActionNode(name,config){

        }

        /**
         * @brief 输入输出参数，节点提供的输入输出参数名及类型申明
         * delay：延迟时间
         * @result 输入输出参数列表
        */
        PortsList CSetTrigger::providedPorts(){
            return { OutputPort<uint32_t>("delay")};
        }

        /**
         * @brief 定时回调，将延迟时间赋给输出参数
         * @result 总是返回SUCCESS
        */
        NodeStatus CSetTrigger::tick()
        {
            uint32_t delay = info::CParam::m_sCurStage.trigger.param1<0?0:(info::CParam::m_sCurStage.trigger.param1*1000);
            setOutput<int>("delay",delay);
            return NodeStatus::SUCCESS;
            
        }

    }
}