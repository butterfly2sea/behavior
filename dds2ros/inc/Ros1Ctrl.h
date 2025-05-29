#ifndef ROS1CTRL_ZYZN_COMMU_H
#define ROS1CTRL_ZYZN_COMMU_H

#include"BaseCtrl.h"
#include<ros/ros.h>
#include<ros/forwards.h>
#include<geometry_msgs/Pose.h>

#include"custom_msgs/CommandRequest.h"
#include"custom_msgs/SimpleVehicle.h"
#include"custom_msgs/CommandResponse.h"

/*
* ROS1通信类，用于ROS信息的发布及获取，获取到相关Ros消息后触发对应信号
*/

namespace zyzn
{
    
    namespace commu
    {
        class CRos1Ctrl : public CBaseCtrl{
            public:
			/*
			* 发布订阅话消息队列长度
			*/
            enum EQueSize{
                AuxiPubSz = 20,    //辅助消息发布队列长度
                AuxiSubSz = 10,    //辅助消息订阅队列长度
                VehiPusSz = 10,    //简单飞控消息发布队列长度
                VehiSubSz = 20,    //简单飞控消息发布队列长度
                RespSubSz = 10,    //回复消息订阅队列长度
                CmdPubSz = 10,     //控制命令发布队列长度
                LineAreaPubSz = 20, //航线消息发布队列长度
                OptSubSz = 10,
                OptPubSz = 20
            };

           class COptInfo{
            public:
            COptInfo(dm::uint8 id);
            void rcvOptInfo(const geometry_msgs::Pose::ConstPtr &info);
            private:
            CRos1Ctrl* m_ros1;
            dm::uint8 m_id;
           };

            CRos1Ctrl();
            void sendCmdInfo(const CCommApi::SCmdInfo& info);
            void sendNavlineArea(const std::vector<CCommApi::SWayPoint> &listInfo,std::string fileName,CCommApi::ESendLineArea type,dm::uint8 dstId=255);
            void sendVehicleSimple(const CCommApi::SVehicleSimple& info);
            void sendAuxiInfo(const CCommApi::SCmdInfo& info);
            void sendOptPose(dm::uint8 id,const SOptPose &info);
            static void auxiCB(const custom_msgs::CommandRequest::ConstPtr &auxi);
            static void vehiSimCB(const custom_msgs::SimpleVehicle::ConstPtr &vehi);
            static void respCB(const custom_msgs::CommandResponse::ConstPtr &resp);
            static void optPoseCB(const geometry_msgs::Pose::ConstPtr &info,const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), const ros::TransportHints& transport_hints = ros::TransportHints());
            void init(int argc,char **argv,const char *nodeName);
            void optInfoInit(int argc,char **argv,const char *nodeName);
            inline ros::Publisher & getAuxiPub(const dm::uint8 & id);
            inline ros::Publisher & getVehiPub(const dm::uint8 & id);
            void addOptPoseSub(dm::uint8 id);

            dm::uint8 getSelfId();
            private:
            ros::Subscriber m_vehSimSub; //飞控消息订阅对象
            ros::Subscriber m_auxiSub;   //辅助消息订阅对象
            ros::Subscriber m_respSub;   //回复消息订阅对象
            ros::Publisher m_attactPub;  //attact publisher
            ros::Publisher m_cmdPub;     //控制命令发布对象
            ros::Publisher m_lineAreaPub;//航线发布对象
            std::map<dm::uint8,ros::Publisher> m_vehSimPubs;//飞控消息发布对象,key:id
            std::map<dm::uint8,ros::Publisher> m_auxiPubs;  //辅助消息发布对象,key:id
            ros::NodeHandle *m_nh;
            ros::Publisher m_optPosePub; //光学定位信息发布对象
            std::map<dm::uint8,ros::Subscriber> m_optPoseSubs;//光学定位信息订阅对象

        };

    } // namespace commu
    
} // namespace zyzn

#endif