#include"Ros1Ctrl.h"
#include"custom_msgs/NavlineArea.h"
#include<mavros_msgs/Waypoint.h>
#include<mavros_msgs/CommandBool.h>

namespace zyzn
{
    namespace commu
    {
        static bool isInit = false;
        static const char* topicNames[ETopicType::TpTypeCount] ={
            "dds2ros/command/request",
            "ros2dds/command/response",
            "ros2dds/set/navlinearea",
            "ros2dds/information/ancillary",
            "ros2dds/information/simple_vehicle",
            "vrpn_client_node/droneyee%d/pose"
        };
        
        static CRos1Ctrl* gros1Ctrl=nullptr;
        static bool rcvSelfId = false;

        CRos1Ctrl::COptInfo::COptInfo(dm::uint8 id):
        m_id(id){

        }

        void CRos1Ctrl::COptInfo::rcvOptInfo(const geometry_msgs::Pose::ConstPtr &info){
            gros1Ctrl->optPoses()[m_id].qw = info->orientation.w*Eq;
            gros1Ctrl->optPoses()[m_id].qx = info->orientation.x*Eq;
            gros1Ctrl->optPoses()[m_id].qy = info->orientation.y*Eq;
            gros1Ctrl->optPoses()[m_id].qz = info->orientation.z*Eq;
            gros1Ctrl->optPoses()[m_id].x = info->position.x * Ep;
            gros1Ctrl->optPoses()[m_id].y = info->position.y * Ep;
            gros1Ctrl->optPoses()[m_id].z = info->position.z * Ep;
            gros1Ctrl->sig_rcvOptPose(m_id,gros1Ctrl->optPoses()[m_id]);
        }

        CRos1Ctrl::CRos1Ctrl():CBaseCtrl()
        {

        }
        void CRos1Ctrl::sendCmdInfo(const CCommApi::SCmdInfo& info){
           custom_msgs::CommandRequest cmdReq;
            cmdReq.srcId=info.srcId;
            cmdReq.dstId=info.dstId;
            cmdReq.grpId=info.grpId;
            cmdReq.type=info.type;
            cmdReq.param0=info.param0;
            cmdReq.param1=info.param1;
            cmdReq.param2=info.param2;
            cmdReq.param3=info.param3;
            cmdReq.param4=info.param4;
            cmdReq.fparam5=info.fparam5;
            cmdReq.fparam6=info.fparam6;
            cmdReq.fparam7=info.fparam7;
            cmdReq.fparam8=info.fparam8;
            m_cmdPub.publish(cmdReq); 
            ROS_INFO_STREAM("\033[31m 发布custom_msgs/CommandRequest类型消息: \033[0m");
            ROS_INFO_STREAM("srcId:" << cmdReq.srcId);
            ROS_INFO_STREAM("dstId:" << cmdReq.dstId);
            ROS_INFO_STREAM("grpId:" << cmdReq.grpId);
            ROS_INFO_STREAM("type:" << cmdReq.type);
            ROS_INFO_STREAM("param0:" << cmdReq.param0);
            ROS_INFO_STREAM("param1:" << cmdReq.param1);
            ROS_INFO_STREAM("param2:" << cmdReq.param2);
            ROS_INFO_STREAM("param3:" << cmdReq.param3);
            ROS_INFO_STREAM("param4:" << cmdReq.param4);
            ROS_INFO_STREAM("fparam5:" << cmdReq.fparam5);
            ROS_INFO_STREAM("fparam6:" << cmdReq.fparam6);
            ROS_INFO_STREAM("fparam7:" << cmdReq.fparam7);
            ROS_INFO_STREAM("fparam8:" << cmdReq.fparam8);
        }
        void CRos1Ctrl::sendNavlineArea(const std::vector<CCommApi::SWayPoint> &listInfo,std::string fileName,CCommApi::ESendLineArea type,dm::uint8 dstId){
            custom_msgs::NavlineArea lineArea;
            lineArea.type = type;
            lineArea.dstId = dstId;
            lineArea.fileName = fileName;
            lineArea.line.waypoints.reserve(listInfo.size());
            for(int i=0;i<listInfo.size();++i){
                mavros_msgs::Waypoint wp;
                wp.x_lat = listInfo[i].lat;
                wp.y_long = listInfo[i].lon;
                wp.z_alt = listInfo[i].alt;
                lineArea.line.waypoints.push_back(wp);
            }
            m_lineAreaPub.publish(lineArea);
        }
        void CRos1Ctrl::sendVehicleSimple(const CCommApi::SVehicleSimple& info){
            if(info.id != m_selfId){
                custom_msgs::SimpleVehicle vehi;
                vehi.synTime=info.synTime;
                vehi.grpId=info.grpId;
                vehi.id=info.id;
                vehi.grpMode=info.grpMode;
                vehi.mode=info.mode;
                vehi.status=info.status;
                vehi.roll=info.roll;
                vehi.pitch=info.pitch;
                vehi.yaw=info.yaw;
                vehi.x=info.x;
                vehi.y=info.y;
                vehi.z=info.z;
                vehi.vx=info.vx;
                vehi.vy=info.vy;
                vehi.vz=info.vz;
                vehi.airSpd=info.airSpd;
                vehi.atcAng=info.atcAng;
                vehi.sldAng=info.sldAng;
                vehi.lon=info.lon;
                vehi.lat=info.lat;
                vehi.alt=info.alt;
                vehi.vol=info.vol;
                getVehiPub(info.id).publish(vehi);
            }
            
        }
        void CRos1Ctrl::sendAuxiInfo(const CCommApi::SCmdInfo& info){
            if(info.srcId != m_selfId){
                custom_msgs::CommandRequest cmdReq;
                cmdReq.srcId=info.srcId;
                cmdReq.dstId=info.dstId;
                cmdReq.grpId=info.grpId;
                cmdReq.type=info.type;
                cmdReq.param0=info.param0;
                cmdReq.param1=info.param1;
                cmdReq.param2=info.param2;
                cmdReq.param3=info.param3;
                cmdReq.param4=info.param4;
                cmdReq.fparam5=info.fparam5;
                cmdReq.fparam6=info.fparam6;
                cmdReq.fparam7=info.fparam7;
                cmdReq.fparam8=info.fparam8;
                getAuxiPub(info.srcId).publish(cmdReq);
            }
            
        }
        void CRos1Ctrl::sendOptPose(dm::uint8 id,const SOptPose &info){
            geometry_msgs::Pose pose;
            pose.orientation.w = info.qw/Eq;
            pose.orientation.x = info.qx/Eq;
            pose.orientation.y = info.qy/Eq;
            pose.orientation.z = info.qz/Eq;
            pose.position.x = info.x/Ep;
            pose.position.y = info.y/Ep;
            pose.position.z = info.z/Ep;
            m_optPosePub.publish(pose);
        }

      

        void CRos1Ctrl::auxiCB(const custom_msgs::CommandRequest::ConstPtr &auxi){
            
            dm::uint8 id = auxi->dstId;
            gros1Ctrl->auxiInfos()[id].srcId=auxi->srcId;
            gros1Ctrl->auxiInfos()[id].dstId=auxi->dstId;
            gros1Ctrl->auxiInfos()[id].grpId=auxi->grpId;
            gros1Ctrl->auxiInfos()[id].type=(CCommApi::ECmd)auxi->type;
            gros1Ctrl->auxiInfos()[id].param0=auxi->param0;
            gros1Ctrl->auxiInfos()[id].param1=auxi->param1;
            gros1Ctrl->auxiInfos()[id].param2=auxi->param2;
            gros1Ctrl->auxiInfos()[id].param3=auxi->param3;
            gros1Ctrl->auxiInfos()[id].param4=auxi->param4;
            gros1Ctrl->auxiInfos()[id].fparam5=auxi->fparam5;
            gros1Ctrl->auxiInfos()[id].fparam6=auxi->fparam6;
            gros1Ctrl->auxiInfos()[id].fparam7=auxi->fparam7;
            gros1Ctrl->auxiInfos()[id].fparam8=auxi->fparam8;

            gros1Ctrl->sig_rcvAuxi(gros1Ctrl->auxiInfos()[id]);
        }
        void CRos1Ctrl::vehiSimCB(const custom_msgs::SimpleVehicle::ConstPtr &vehi){
            dm::uint8 id = vehi->id;
            gros1Ctrl->vehicleSimples()[id].synTime=vehi->synTime;
            gros1Ctrl->vehicleSimples()[id].grpId=vehi->grpId;
            gros1Ctrl->vehicleSimples()[id].id=vehi->id;
            gros1Ctrl->vehicleSimples()[id].grpMode=vehi->grpMode;
            gros1Ctrl->vehicleSimples()[id].mode=vehi->mode;
            gros1Ctrl->vehicleSimples()[id].status=vehi->status;
            gros1Ctrl->vehicleSimples()[id].roll=vehi->roll;
            gros1Ctrl->vehicleSimples()[id].pitch=vehi->pitch;
            gros1Ctrl->vehicleSimples()[id].yaw=vehi->yaw;
            gros1Ctrl->vehicleSimples()[id].x=vehi->x;
            gros1Ctrl->vehicleSimples()[id].y=vehi->y;
            gros1Ctrl->vehicleSimples()[id].z=vehi->z;
            gros1Ctrl->vehicleSimples()[id].vx=vehi->vx;
            gros1Ctrl->vehicleSimples()[id].vy=vehi->vy;
            gros1Ctrl->vehicleSimples()[id].vz=vehi->vz;
            gros1Ctrl->vehicleSimples()[id].airSpd=vehi->airSpd;
            gros1Ctrl->vehicleSimples()[id].atcAng=vehi->atcAng;
            gros1Ctrl->vehicleSimples()[id].sldAng=vehi->sldAng;
            gros1Ctrl->vehicleSimples()[id].lon=vehi->lon;
            gros1Ctrl->vehicleSimples()[id].lat=vehi->lat;
            gros1Ctrl->vehicleSimples()[id].alt=vehi->alt;
            gros1Ctrl->vehicleSimples()[id].vol=vehi->vol;
            gros1Ctrl->sig_rcvVehicleSimple(gros1Ctrl->vehicleSimples()[id]);
        }
        void CRos1Ctrl::respCB(const custom_msgs::CommandResponse::ConstPtr &resp){
            gros1Ctrl->sig_rcvResp(resp->id,resp->src,resp->cmd,resp->subcmd,resp->rslt,resp->rslt);       
        }
        void CRos1Ctrl::optPoseCB(const geometry_msgs::Pose::ConstPtr &info,const ros::VoidConstPtr& tracked_object, const ros::TransportHints& transport_hints){
            dm::uint8 *pId = (dm::uint8*)tracked_object.get();
            if(pId){
                gros1Ctrl->optPoses()[*pId].qw = info->orientation.w*Eq;
                gros1Ctrl->optPoses()[*pId].qx = info->orientation.x*Eq;
                gros1Ctrl->optPoses()[*pId].qy = info->orientation.y*Eq;
                gros1Ctrl->optPoses()[*pId].qz = info->orientation.z*Eq;
                gros1Ctrl->optPoses()[*pId].x = info->position.x * Ep;
                gros1Ctrl->optPoses()[*pId].y = info->position.y * Ep;
                gros1Ctrl->optPoses()[*pId].z = info->position.z * Ep;
                gros1Ctrl->sig_rcvOptPose(*pId,gros1Ctrl->optPoses()[*pId]);
            }
        }
        void CRos1Ctrl::init(int argc,char **argv,const char *nodeName){
            if(!isInit){
                ros::init(argc, argv, nodeName);
                m_nh = new ros::NodeHandle();
                gros1Ctrl = this;
                //fauxiCb = &CRos1Ctrl::auxiCB;
                              
                m_respSub = m_nh->subscribe<custom_msgs::CommandResponse>(topicNames[TpCmdRes],RespSubSz,&CRos1Ctrl::respCB);
                //m_auxiPub = m_nh.advertise<custom_msgs::CommandRequest>("",AuxiPubSz);
                m_cmdPub = m_nh->advertise<custom_msgs::CommandRequest>(topicNames[TpCmdReq],CmdPubSz);
                m_lineAreaPub = m_nh->advertise<custom_msgs::NavlineArea>(topicNames[TpSetLineArea],LineAreaPubSz);
                //m_vehSimPub = m_nh.advertise<custom_msgs::SimpleVehicle>("",VehiPusSz);

                isInit = true;

            }
        }
        void CRos1Ctrl::optInfoInit(int argc,char **argv,const char *nodeName){
            if(!isInit){
                ros::init(argc, argv, nodeName);
                m_nh = new ros::NodeHandle();
                gros1Ctrl = this;
                //fauxiCb = &CRos1Ctrl::auxiCB;

                isInit = true;

            }
        }
        ros::Publisher & CRos1Ctrl::getAuxiPub(const dm::uint8 & id){
                
                if(m_auxiPubs.find(id) == m_auxiPubs.end() && m_nh){
                    std::string name(topicNames[TpSetAuxi]);
                    char strId[NumLen] = {0};
                    snprintf(strId,NumLen,"%d",id);
                    name += strId;
                    m_auxiPubs[id] = m_nh->advertise<custom_msgs::CommandRequest>(name,AuxiPubSz);
                }
                return m_auxiPubs[id];
        }

        ros::Publisher & CRos1Ctrl::getVehiPub(const dm::uint8 & id){
            if(m_vehSimPubs.find(id) == m_vehSimPubs.end() && m_nh){
                    std::string name(topicNames[TpSetVehicle]);
                    char strId[NumLen] = {0};
                    snprintf(strId,NumLen,"%d",id);
                    name += strId;
                    m_vehSimPubs[id] = m_nh->advertise<custom_msgs::SimpleVehicle>(name,VehiPusSz);
                }
                return m_vehSimPubs[id];
        }
        void CRos1Ctrl::addOptPoseSub(dm::uint8 id){
            auto it = m_optPoseSubs.find(id);
            if(it == m_optPoseSubs.end()){
                
                char name[NameLen] = {0};
                snprintf(name,NameLen,topicNames[TpOpticalPose],id);
                COptInfo *pInfo = new COptInfo(id);
                m_optPoseSubs[id] = m_nh->subscribe<geometry_msgs::Pose,COptInfo>(name,OptSubSz,&COptInfo::rcvOptInfo,pInfo);
            }
        }
      
        dm::uint8 CRos1Ctrl::getSelfId(){
            while(m_selfId==0)
            {
            if(m_nh){
                if(0 != m_selfId)
                    return m_selfId;
                ros::ServiceClient getIdClient = m_nh->serviceClient<mavros_msgs::CommandBool>("inner/get/id_vehicle");
                mavros_msgs::CommandBool cmdGetId;
                cmdGetId.request.value = true;
                if( getIdClient.call(cmdGetId) &&
                    cmdGetId.response.success){
                    m_selfId = cmdGetId.response.result;
                    std::string name(topicNames[TpSetAuxi]);
                    char strId[NumLen] = {0};
                    snprintf(strId,NumLen,"%d",m_selfId);
                    name += strId;
                    m_auxiSub = m_nh->subscribe<custom_msgs::CommandRequest>(name,AuxiSubSz,CRos1Ctrl::auxiCB);
                    ROS_INFO_STREAM("subscribe self auxi TopicName:" << name);

                    name = topicNames[TpSetVehicle];
                    name += strId;
                    m_vehSimSub = m_nh->subscribe<custom_msgs::SimpleVehicle>(name,VehiSubSz,CRos1Ctrl::vehiSimCB);
                    ROS_INFO_STREAM("subscribe self vehicle TopicName:" << name);
                    ROS_INFO_STREAM("getself id:" << m_selfId);

                    char nameOpt[NameLen] = {0};
                    snprintf(nameOpt,NameLen,topicNames[TpOpticalPose],m_selfId);
                    m_optPosePub = m_nh->advertise<geometry_msgs::Pose>(nameOpt,OptPubSz);
                }
            }
                return m_selfId;
            }
            
        }

    } // namespace commu
    
} // namespace zyzn
