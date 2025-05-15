#include "../../include/check/CheckStartTask.hpp"

namespace zyzn{
    namespace check{
        CCheckStartTask::CCheckStartTask(const std::string& name, const NodeConfig& config):SyncActionNode(name,config),
        m_type(DelayTime),m_time(0),m_start(0),m_rcvAck(false){

        }

        PortsList CCheckStartTask::providedPorts() {
            return { InputPort<uint32_t>("delay")};
        }


        NodeStatus CCheckStartTask::tick(){
            time_t tn;
            m_time = 0;
            getInput<uint32_t>("delay",m_time);
            m_time/=1000;
            switch (m_type)
            {
            case FixTime:
                /* code */
                if(time(nullptr) >= m_time)
                    return NodeStatus::SUCCESS;
                break;
            case DelayTime:
                if(0==m_start){
                    m_start = time(nullptr);
                }
                tn = time(nullptr);
                if(tn - m_start >= m_time){
                    m_start = 0;
                    return NodeStatus::SUCCESS;
                }
                break;
            case Manual:
                if(m_rcvAck)
                    return NodeStatus::SUCCESS;
                break;
            case Auto:
                return NodeStatus::SUCCESS;
            break;
            
            default:
                break;
            }
            return NodeStatus::FAILURE;
        }
    }
}