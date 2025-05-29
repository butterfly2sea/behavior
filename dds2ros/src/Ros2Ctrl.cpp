#include<time.h>
#include<sys/time.h>
#include<fstream>
#include <chrono>
#include <fastdds/dds/log/Log.hpp>
#include <log/Logger.hpp>
#include"Ros2Ctrl.h"
#include "rclcpp/rclcpp.hpp"
#include<StageHead.hpp>

static std::ofstream glogFile;
static char glocBuf[64] = {0};
static bool gwtid = false;
using namespace std::chrono;
namespace zyzn{
    namespace commu
    {
        static const char* topicNames[ETopicType::TpTypeCount] ={
            "outer/set/attack_object_designate",
            "outer/command/request",
            "outer/command/response",
            "inner/information/object_computation",
            "outer/information/vrpn",
            "outer/set/task_stage",
            "outer/information/simple_vehicle",
            "outer/information/status_task",
            "inner/information/warnning",
            "inner/information/simple_vehicle",
            "outer/information/object_computation",
            "outer/set/stage_info_json",
            "outer/information/auto_navline"
        };

         static CRos2Ctrl* gros1Ctrl=nullptr;
        using std::placeholders::_1;
        CRos2Ctrl::CRos2Ctrl(const char *nodeName):CBaseCtrl(),Node(nodeName){
           
        }
        void CRos2Ctrl::sendAttactInfo(const CCommApi::SAttackObjectDesignate& info)
        {
            custom_msgs::msg::ObjectAttackDesignate attact;
            attact.grpid = info.grpid;
            attact.ids = info.ids;
            attact.type = info.type;
            attact.objs.id=info.objs.id;
            //attact.objs.objid=info.objs.objid;
            attact.objs.subcls=info.objs.subcls;
            attact.objs.x=info.objs.x;
            attact.objs.y=info.objs.y;
            attact.objs.z=info.objs.z;
            attact.objs.vx=info.objs.vx;
            attact.objs.vy=info.objs.vy;
            attact.objs.vz=info.objs.vz;
            //
            m_AttactPub->publish(attact);
            txtLog().info(THISMODULE "attact Publishing grpid:%d type:%d objid:%d x:%d y:%d z:%d",attact.grpid,
            attact.type,attact.objs.id,attact.objs.x,attact.objs.y,attact.objs.z);
        }

        void CRos2Ctrl::sendCmdInfo(const CCommApi::SCommandRequest& info){
            custom_msgs::msg::CommandRequest cmdReq;
            cmdReq.src=info.srcid;
            cmdReq.dst=info.dstid;
            cmdReq.grp=info.grpid;
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
            m_cmdPub->publish(cmdReq);
            txtLog().info(THISMODULE "cmdReq Publishing grpid:%d type:%d param0:%d param1:%d param2:%d param3:%d param4:%d fparam5:%f fparam6:%f fparam7:%f fparam8:%f",
            cmdReq.grp,cmdReq.type,cmdReq.param0,cmdReq.param1,cmdReq.param2,cmdReq.param3,cmdReq.param4,cmdReq.fparam5,cmdReq.fparam6,cmdReq.fparam7,
            cmdReq.fparam8);
        }
       
        void CRos2Ctrl::sendVehicleSimple(const CCommApi::SSimpleVehicle& info){

            custom_msgs::msg::SimpleVehicle simVehicle;
            simVehicle.time = info.time;      
            simVehicle.grp = info.grp;
            simVehicle.id = info.id;
            simVehicle.bat = info.bat;
            simVehicle.fix = info.fix;
            simVehicle.volt = info.volt;
            simVehicle.sate = info.sate;
            simVehicle.lock = info.lock;
            simVehicle.flymd = info.flymd;
            simVehicle.roll = info.roll;
            simVehicle.pitch = info.pitch;
            simVehicle.yaw = info.yaw;
            simVehicle.x = info.x;
            simVehicle.y = info.y;
            simVehicle.z = info.z;
            simVehicle.vx = info.vx;
            simVehicle.vy = info.vy;
            simVehicle.vz = info.vz;
            simVehicle.airspd = info.airspd;
            simVehicle.lon = info.lon;
            simVehicle.lat = info.lat;
            simVehicle.alt = info.alt;
            simVehicle.radz = info.radz;
            m_outterVehSimPub->publish(simVehicle);
            
            }

        void CRos2Ctrl::sendTaskStage(const CCommApi::STaskStage& info){
            custom_msgs::msg::TaskStage taskStage;
            taskStage.head.grpid = info.head.grpid;             //分组编号
			taskStage.head.ids   = info.head.ids;               //同分组内全部id号
			taskStage.head.stage = info.head.stage;             //任务阶段
			taskStage.head.tag   = info.head.tag;               //阶段标识
            taskStage.scopetype=info.scopetype;
            taskStage.line.points.resize(info.linx.size());

            for(int i=0;i<info.linx.size();i++){
                taskStage.line.points[i].x =info.linx[i];
                taskStage.line.points[i].y =info.liny[i];
                taskStage.line.points[i].z =info.linz[i];
            }
            taskStage.maxspd=info.maxspd;
            taskStage.trigger.type=info.trigger.type;
            taskStage.trigger.param1=info.trigger.param1;
            taskStage.trigger.param2=info.trigger.param2;
            taskStage.trigger.param3=info.trigger.param3;
            taskStage.trigger.param4=info.trigger.param4;
            
            taskStage.form.type=info.form.type;
            taskStage.form.param1=info.form.param1;
            taskStage.form.param2=info.form.param2;
            taskStage.form.param3=info.form.param3;
            taskStage.form.param4=info.form.param4;
            taskStage.formoffset.points.resize(info.formOffsetx.size());
            for(int i=0;i<info.formOffsetx.size();i++){
                taskStage.formoffset.points[i].x =info.formOffsetx[i];
                taskStage.formoffset.points[i].y =info.formOffsety[i];
                taskStage.formoffset.points[i].z =info.formOffsetz[i];
            }
            taskStage.objs.type =info.objs.type;
            taskStage.objs.path =info.objs.path;
            taskStage.objs.cls =info.objs.cls;
            taskStage.objs.subcls =info.objs.subcls;
            m_taskStagePub->publish(taskStage);
        }

        void CRos2Ctrl::sendStatusTask(const CCommApi::SStatusTask& info){
            custom_msgs::msg::StatusTask statusTask;
            statusTask.id=info.id;            //飞机id
			statusTask.stage=info.stage;	  //任务阶段
			statusTask.tag=info.tag;          //阶段标识 0:前突,1:搜索,2:跟踪打击,3:回收
			statusTask.status=info.status;    //任务状态
			statusTask.dstwaypt=info.dstwaypt;//目标航点id
			statusTask.diswaypt=info.diswaypt;//距离目标航点距离
			statusTask.disobj=info.disobj;    //距离目标距离
			statusTask.obj=info.obj;          //目标id
           
        }
        
        void CRos2Ctrl::sendOptPose(dm::uint8 id,const CCommApi::SOpticalPose &info){
            geometry_msgs::msg::Pose pose;
            pose.orientation.x = info.roll;
            pose.orientation.y = info.pitch;
            pose.orientation.z = info.yaw;
            pose.position.x = info.x;
            pose.position.y = info.y;
            pose.position.z = info.z;
            m_optPosePub->publish(pose);

        }

        void CRos2Ctrl::sendObjectComputation(const CCommApi::SObjectReport& info){
            custom_msgs::msg::ObjectComputation objCom;
            objCom.objs.resize(1);
            if(objCom.objs.size()){
                objCom.objs[0].id=info.id;
                //objCom.objs[0].objid=info.objid;
                objCom.objs[0].cls=info.cls;
                objCom.objs[0].subcls=info.subcls;
                objCom.objs[0].score=info.score;
                objCom.objs[0].x=info.x;
                objCom.objs[0].y=info.y;
                objCom.objs[0].z=info.z;
                objCom.objs[0].vx=info.vx;
                objCom.objs[0].vy=info.vy;
                objCom.objs[0].vz=info.vz;
                m_outterObjPub->publish(objCom);
            }

        }

        void CRos2Ctrl::sendStageSet(const std::string &info){
            std_msgs::msg::String stg;
            stg.data = info;
            m_stageSetPub->publish(stg);
        }

      
        void CRos2Ctrl::vehiSimCB(const custom_msgs::msg::SimpleVehicle::SharedPtr info){
            if(0 == info->id)
                return;
            if((CBaseCtrl::InvalId==m_selfId) && (info->id<CBaseCtrl::MaxId)){
                setSelfId(info->id);
            }
            static system_clock::time_point preSend = system_clock::now();
            system_clock::time_point nt = system_clock::now();
            duration<int,std::nano> exp = nt - preSend;
            if(exp.count()/1e6 < 50){
                return;
            }
            preSend = nt;
            
            CCommApi::SSimpleVehicle simVehicle;
            simVehicle.time=info->time;      
            simVehicle.grp=info->grp;
            simVehicle.id=info->id;
            simVehicle.bat=info->bat;
            simVehicle.fix=info->fix;
            simVehicle.volt=info->volt;
            simVehicle.sate=info->sate;
            simVehicle.lock=info->lock;
            simVehicle.flymd=info->flymd;
            simVehicle.roll=info->roll;
            simVehicle.pitch=info->pitch;
            simVehicle.yaw=info->yaw;
            simVehicle.x = info->x;
            simVehicle.y = info->y;
            simVehicle.z = info->z;
            simVehicle.vx=info->vx;
            simVehicle.vy=info->vy;
            simVehicle.vz=info->vz;
            simVehicle.airspd=info->airspd;
            simVehicle.lon=info->lon;
            simVehicle.lat=info->lat;
            simVehicle.alt=info->alt;
            simVehicle.radz=info->radz;
            
            sig_rcvVehicleSimple(simVehicle);

        }
        void CRos2Ctrl::respCB(const custom_msgs::msg::CommandResponse::SharedPtr resp){
             this->sig_rcvResp(resp->id,resp->src,resp->type,resp->status,resp->rslt);  

        }

        void CRos2Ctrl::taskStageCB(const custom_msgs::msg::TaskStage::SharedPtr info){
            CCommApi::STaskStage taskStage;
            taskStage.head.grpid = info->head.grpid;   //分组编号
			taskStage.head.ids   = info->head.ids;     //同分组内全部id号
			taskStage.head.stage = info->head.stage;   //任务阶段
			taskStage.head.tag   = info->head.tag;     //阶段标识
            taskStage.scopetype=info->scopetype;

            for(int i=0;i<info->line.points.size();i++){
                taskStage.linx.push_back(info->line.points[i].x);
                taskStage.liny.push_back(info->line.points[i].y);
                taskStage.linz.push_back(info->line.points[i].z);
            }
            taskStage.maxspd=info->maxspd;
            taskStage.trigger.type=info->trigger.type;
            taskStage.trigger.param1=info->trigger.param1;
            taskStage.trigger.param2=info->trigger.param2;
            taskStage.trigger.param3=info->trigger.param3;
            taskStage.trigger.param4=info->trigger.param4;
            
            taskStage.form.type=info->form.type;
            taskStage.form.param1=info->form.param1;
            taskStage.form.param2=info->form.param2;
            taskStage.form.param3=info->form.param3;
            taskStage.form.param4=info->form.param4;
            for(int i=0;i<info->formoffset.points.size();i++){
                taskStage.formOffsetx.push_back(info->formoffset.points[i].x);
                taskStage.formOffsety.push_back(info->formoffset.points[i].y);
                taskStage.formOffsetz.push_back(info->formoffset.points[i].z);
            }
            taskStage.objs.type =info->objs.type;
            taskStage.objs.path =info->objs.path;
            taskStage.objs.cls =info->objs.cls;
            taskStage.objs.subcls =info->objs.subcls;
        }

        void CRos2Ctrl::statusTaskCB(const custom_msgs::msg::StatusTask::SharedPtr info){
            CCommApi::SStatusTask statusTask;
            statusTask.id=info->id;          //飞机id
			statusTask.stage=info->stage;	   //任务阶段
			statusTask.tag=info->tag;         //阶段标识 0:前突,1:搜索,2:跟踪打击,3:回收
			statusTask.status=info->status;      //任务状态
			statusTask.dstwaypt=info->dstwaypt;   //目标航点id
			statusTask.diswaypt=info->diswaypt;   //距离目标航点距离
			statusTask.disobj=info->disobj;     //距离目标距离
			statusTask.obj=info->obj;        //目标id
            sig_rcvStatusTask(statusTask);

        }

        void CRos2Ctrl::vehiwarningCB(const custom_msgs::msg::VehiWarning::SharedPtr info){
            CCommApi::SVehiWarning vehiwarning;
            //vehiwarning.id=info->id;     //飞机id
			vehiwarning.src=info->src;    //告警信息来源 0:飞控,1:光电,2:雷达,3:武器
			vehiwarning.tag=info->tag;   //告警信息类别
			vehiwarning.level=info->level; //告警级别
			vehiwarning.desc=info->desc;  //告警信息描述 
            sig_rcvWarning(vehiwarning);
        }

        void CRos2Ctrl::objectComputationCB(const custom_msgs::msg::ObjectComputation::SharedPtr info){
            CCommApi::SObjectReport objReport;
            const custom_msgs::msg::ObjectLocation::SharedPtr objLocation;
            for(int i=0;i<info->objs.size();i++){
                objReport.objid = info->objs[i].id;        //objid
			    objReport.cls=info->objs[i].cls;	     //目标分类
			    objReport.subcls=info->objs[i].subcls;   //目标子分类
                objReport.score=info->objs[i].score;   //object score
			    objReport.x=info->objs[i].x;         //估计目标locx
			    objReport.y=info->objs[i].y;         //估计目标locy
			    objReport.z=info->objs[i].z;         //估计目标locz
			    objReport.vx=info->objs[i].vx;        //估计目标速度vx
			    objReport.vy=info->objs[i].vy;        //估计目标速度vy
			    objReport.vz=info->objs[i].vz;        //估计目标速度vz

                sig_rcvObjComputation(objReport);
            }
            
        }

        void CRos2Ctrl::autoNavlineCB(const geometry_msgs::msg::Polygon::SharedPtr info){
            CCommApi::SAutoNvaline autonva;
            autonva.x_lat.reserve(info->points.size());
            txtLog().info(THISMODULE "autoNavlineCB size:%d",info->points.size());
            autonva.type = 0;
                //info->line.points->x;
                for(int i=0;i<info->points.size();i++)
                {
                    autonva.x_lat.push_back(info->points[i].x*1e3);        //objid
			        autonva.y_lon.push_back(info->points[i].y*1e3);	     //目标分类
			        autonva.z_alt.push_back(info->points[i].z*1e3);   //目标子分类
                }
                sig_rcvAutoNva(autonva);
            
        }

        
        void CRos2Ctrl::init(){
            
            m_cmdPub = this->create_publisher<custom_msgs::msg::CommandRequest>(topicNames[TpCommandRequest],CmdPubSz);
           
            m_outterVehSimPub = this->create_publisher<custom_msgs::msg::SimpleVehicle>(topicNames[TpSimpleVehicle],rclcpp::SensorDataQoS());

            m_AttactPub = this->create_publisher<custom_msgs::msg::ObjectAttackDesignate>(topicNames[TpAttackObjectDesignate],AttackPubSz);

            m_optPosePub=this->create_publisher<geometry_msgs::msg::Pose>(topicNames[TpOpticalPose],rclcpp::SensorDataQoS());
            
            m_outterObjPub=this->create_publisher<custom_msgs::msg::ObjectComputation>(topicNames[TpSpecialObjReport],ObjectReportSz);

            m_stageSetPub=this->create_publisher<std_msgs::msg::String>(topicNames[TpStageSet],StageSetSz);
            
            gros1Ctrl = this;
            m_taskStagePub = this->create_publisher<custom_msgs::msg::TaskStage>(topicNames[TpTaskStage],TaskPubSz);

            
            //
            m_statusTaskSub = this->create_subscription<custom_msgs::msg::StatusTask>(topicNames[TpStatusTask],StatusTaskPubSz,std::bind(&CRos2Ctrl::statusTaskCB,this,_1));

            m_innerObjSub = this->create_subscription<custom_msgs::msg::ObjectComputation>(topicNames[TpObjectReport],rclcpp::SensorDataQoS(),std::bind(&CRos2Ctrl::objectComputationCB,this,_1));

            m_innerVehSimSub = this->create_subscription<custom_msgs::msg::SimpleVehicle>(topicNames[TpInnerSimpleVehicle],rclcpp::SensorDataQoS(),std::bind(&CRos2Ctrl::vehiSimCB,this,_1));
           
            m_respSub = this->create_subscription<custom_msgs::msg::CommandResponse>(topicNames[TpCommandResponse],RespSubSz,std::bind(&CRos2Ctrl::respCB,this,_1));
            
            m_warnSub = this->create_subscription<custom_msgs::msg::VehiWarning>(topicNames[TpVehiWarning],rclcpp::SensorDataQoS(),std::bind(&CRos2Ctrl::vehiwarningCB,this,_1));

            m_autoNavSub = this->create_subscription<geometry_msgs::msg::Polygon>(topicNames[TpAutoLine],rclcpp::SensorDataQoS(),std::bind(&CRos2Ctrl::autoNavlineCB,this,_1));


        }

    }
}
