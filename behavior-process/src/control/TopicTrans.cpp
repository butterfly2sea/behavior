#include <control/TopicTrans.hpp>
#include <info/Param.hpp>
namespace zyzn{
    namespace ctrl{
        std_msgs::msg::String TopicTrans::m_info;
        TopicTrans::TopicTrans(const std::string& name,
        const NodeConfig& conf):SyncActionNode(name, conf),m_pubInfo(nullptr){
            std::string tpcName("formation/task_gen");
            getInput<std::string>("tpc", tpcName);
            m_pubInfo = info::CParam::m_glbNode->create_publisher<std_msgs::msg::String>(tpcName,10);
        }

        PortsList TopicTrans::providedPorts(){
            return {InputPort<std::string>("tpc")};
        }


        NodeStatus TopicTrans::tick(){
            
            if(m_pubInfo){
                
                m_pubInfo->publish(m_info);
            }
            return NodeStatus::SUCCESS;      
        }
    }
}