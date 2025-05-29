#ifndef ROS2CTRL_ZYZN_COMMU_H
#define ROS2CTRL_ZYZN_COMMU_H

#include"BaseCtrl.h"
#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<custom_msgs/msg/command_request.hpp>
#include<custom_msgs/msg/simple_vehicle.hpp>
#include<custom_msgs/msg/command_response.hpp>
#include<custom_msgs/msg/ancillary_information.hpp>
#include<custom_msgs/msg/object_attack_designate.hpp>
#include<custom_msgs/msg/simple_vehicle.hpp>
#include<custom_msgs/msg/task_stage.hpp>
#include<custom_msgs/msg/command_response.hpp>
#include<custom_msgs/msg/status_task.hpp>
#include<custom_msgs/msg/vehi_warning.hpp>
#include<custom_msgs/msg/object_computation.hpp>
#include<custom_msgs/srv/command_bool.hpp>

#include<geometry_msgs/msg/polygon.hpp>


/*
* ROS2通信类，用于ROS信息的发布及获取，获取到相关Ros2消息后触发对应信号
*/

namespace zyzn
{
    
    namespace commu
    {
        class CRos2Ctrl : public CBaseCtrl,public rclcpp::Node{
            public:
			/*
			* 发布订阅话消息队列长度
			*/
            enum EQueSize{
                AuxiPubSz = 20,    //辅助消息发布队列长度
                AuxiSubSz = 10,    //辅助消息订阅队列长度
                VehiPubSz = 10,    //简单飞控消息发布队列长度
                VehiSubSz = 20,    //简单飞控消息发布队列长度
                RespSubSz = 10,    //回复消息订阅队列长度
                CmdPubSz = 10,     //控制命令发布队列长度
                LineAreaPubSz = 20, //航线消息发布队列长度
                OptSubSz = 10,
                OptPubSz = 20,
                AttackPubSz = 10,
                AttackSubSz = 10,
                SimVelPubSz = 10,
                SimVelSubSz = 10,
                TaskPubSz = 10,
                TaskSubSz = 10,
                StatusTaskPubSz = 10,
                StatusTaskSubSz =10,
                ObjectReportSz = 10,
                StageSetSz = 10,
                WarnMsgSz = 10
            };
       
            CRos2Ctrl(const char *nodeName);
            void sendAttactInfo(const CCommApi::SAttackObjectDesignate& info);
            void sendCmdInfo(const CCommApi::SCommandRequest& info);
            void sendVehicleSimple(const CCommApi::SSimpleVehicle& info);
            void sendTaskStage(const CCommApi::STaskStage& info);
            void sendStatusTask(const CCommApi::SStatusTask& info);
            //void sendAuxiInfo(const CCommApi::SCommandRequest& info);
            void sendOptPose(dm::uint8 id,const CCommApi::SOpticalPose &info);
            void sendObjectComputation(const CCommApi::SObjectReport& info);
            void sendStageSet(const std::string &info);

            void vehiSimCB(const custom_msgs::msg::SimpleVehicle::SharedPtr info);
            void respCB(const custom_msgs::msg::CommandResponse::SharedPtr esp);
            void taskStageCB(const custom_msgs::msg::TaskStage::SharedPtr info);
            void statusTaskCB(const custom_msgs::msg::StatusTask::SharedPtr info);
            void vehiwarningCB(const custom_msgs::msg::VehiWarning::SharedPtr info);
            void objectComputationCB(const custom_msgs::msg::ObjectComputation::SharedPtr info);
            void autoNavlineCB(const geometry_msgs::msg::Polygon::SharedPtr info);
            
            void init();

            private:
            rclcpp::Publisher<custom_msgs::msg::ObjectAttackDesignate>::SharedPtr m_AttactPub;//attact information publisher object
            rclcpp::Publisher<custom_msgs::msg::CommandRequest>::SharedPtr m_cmdPub;     //控制命令发布对象
            rclcpp::Publisher<custom_msgs::msg::SimpleVehicle>::SharedPtr m_outterVehSimPub;     //控制命令发布对象
            rclcpp::Publisher<custom_msgs::msg::TaskStage>::SharedPtr m_taskStagePub;     //task命令发布对象
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_optPosePub; //光学定位信息发布对象
            rclcpp::Publisher<custom_msgs::msg::ObjectComputation>::SharedPtr m_outterObjPub;     //object report publish
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_stageSetPub; //log publish

            rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr m_innerVehSimSub; //飞控消息订阅对象
            rclcpp::Subscription<custom_msgs::msg::CommandResponse>::SharedPtr m_respSub;   //回复消息订阅对象
            rclcpp::Subscription<custom_msgs::msg::StatusTask>::SharedPtr m_statusTaskSub;     //task status命令发布对象
            rclcpp::Subscription<custom_msgs::msg::ObjectComputation>::SharedPtr m_innerObjSub;     //object report 
            rclcpp::Subscription<custom_msgs::msg::VehiWarning>::SharedPtr m_warnSub;
            rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr m_autoNavSub;
        };

    } // namespace commu
    
} // namespace zyzn
#endif