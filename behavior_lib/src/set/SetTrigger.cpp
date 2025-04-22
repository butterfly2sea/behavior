#include "../../include/set/SetTrigger.hpp"
#include "../../include/plugin/base_plugin.hpp"

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
            uint32_t delay = plugin::BasePlugin::getValue(info::CParam::getTrigger("delay"))*1000;//秒转换为毫秒
            setOutput<uint32_t>("delay",delay);
            return NodeStatus::SUCCESS;
            
        }

    }
}