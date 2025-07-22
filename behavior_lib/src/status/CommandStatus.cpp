#include "../../include/status/CommandStatus.hpp"
#include "../../include/info/Param.hpp"
#include "../../include/check/CheckWayViaTp.hpp"

namespace zyzn{
    namespace status{
        rclcpp::Publisher<custom_msgs::msg::CommandResponse>::SharedPtr CCommandStatus::m_s_pubResp=nullptr;
        rclcpp::Publisher<custom_msgs::msg::StatusTask>::SharedPtr CCommandStatus::m_s_pubStsTask=nullptr;
        custom_msgs::msg::StatusTask CCommandStatus::m_s_msgStsTsk;
        custom_msgs::msg::CommandResponse CCommandStatus::m_s_msgCmdRsp;
        CCommandStatus::CCommandStatus(const std::string& name,
        const NodeConfig& conf):SyncActionNode(name, conf),m_rspType(CMD_RSP){
            init();
        }

        CCommandStatus::CCommandStatus():SyncActionNode("",NodeConfig()),m_rspType(CMD_RSP){
            init();
        }

        PortsList CCommandStatus::providedPorts(){
            return {
            InputPort<int>("cmd"),
            InputPort<int>("subcmd"),
            InputPort<int>("status"),
            InputPort<int>("rspType"),
            InputPort<std::string>("rslt")};
        }
        
        void CCommandStatus::sendCmdRsp(){
            
            m_s_msgCmdRsp.id = zyzn::info::CParam::vehiId();
            m_s_pubResp->publish(m_s_msgCmdRsp);
        }

        void CCommandStatus::sendTaskStatus(){
            m_s_pubStsTask->publish(m_s_msgStsTsk);
        }

        NodeStatus CCommandStatus::tick(){
            if(CMD_RSP & m_rspType){//发送指令回复
                int src = 0;
                int type = 0;
                int status = 0;
                getInput<int>("cmd",src);
                getInput<int>("subcmd",type);
                getInput<int>("status",status);
                getInput<std::string>("rslt",m_s_msgCmdRsp.rslt);
                m_s_msgCmdRsp.src = src;
                m_s_msgCmdRsp.type = type;
                m_s_msgCmdRsp.status = status;
                sendCmdRsp();
            }
            if(TASK_RSP & m_rspType){//发送任务状态
                m_s_msgStsTsk.dstwaypt = check::CCheckWayViaTp::curNav().id;
                m_s_msgStsTsk.diswaypt = check::CCheckWayViaTp::curNav().dis;
                int status = StsOngoing;
                getInput<int>("status",status);
                m_s_msgStsTsk.status = status;
                sendTaskStatus();               
            }

            return NodeStatus::SUCCESS;
        }

        void CCommandStatus::init(){
            getInput<int>("rspType",m_rspType);
            if(!m_s_pubResp){
                m_s_pubResp = info::CParam::rosNode()->create_publisher<custom_msgs::msg::CommandResponse>
            ("outer/command/response",10);
            }
            if(!m_s_pubStsTask){
                m_s_pubStsTask = info::CParam::rosNode()->create_publisher<custom_msgs::msg::StatusTask>
                ("outer/information/status_task",10);
            }
        }

        void CCommandStatus::initResp(int subCmd,ECtrlType ctrl){
            m_s_msgCmdRsp.id = info::CParam::vehiId();
            m_s_msgCmdRsp.src = ctrl;
            m_s_msgCmdRsp.type = subCmd;
            m_s_msgCmdRsp.rslt = "";
            m_s_msgCmdRsp.status = Failed;
        }

    }
}