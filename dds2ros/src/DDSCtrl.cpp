#include <filesystem>
#include <chrono>
#include <dm/bytes.hpp>
#include <ctime>
#include"AttackObjectDesignatePubSubTypes.hpp"
#include"CommandRequestPubSubTypes.hpp"
#include"CommandResponsePubSubTypes.hpp"
#include"ObjectReportPubSubTypes.hpp"
#include"OpticalPosePubSubTypes.hpp"
#include"ObjectFilterPubSubTypes.hpp"
#include"ParamShortPubSubTypes.hpp"
#include"TaskStagePubSubTypes.hpp"
#include"SimpleVehiclePubSubTypes.hpp"
#include"VehiWarningPubSubTypes.hpp"
#include"StatusTaskPubSubTypes.hpp"
#include"StageHeadPubSubTypes.hpp"
#include"VehiWarningPubSubTypes.hpp"
#include"StageSetPubSubTypes.hpp"
#include"AutoLinePubSubTypes.hpp"
#include"DDSCtrl.h"

//#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/log/FileConsumer.hpp>
#ifndef __linux
#include <Windows.h>
#else
#include <log/Logger.hpp>
#endif


namespace zyzn
{
    namespace commu
    {
        static const char* typeNames[EDDSTopicType::DDS_TpTypeCount] = {
            "AttackObjectDesignate",//跟踪打击指定目标
            "CommandRequest",       //命令请求
            "CommandResponse",      //命令回复
            "ObjectReport",
            "OpticalPose",          //动捕数据，已弃用
            "TaskStage",            //任务阶段，现用于编队分组及偏移设置
            "SimpleVehicle",        //精简飞控消息
            "StatusTask",           //任务状态
            "VehiWarning",          //告警消息
            "SpecialObjReport",
            "StageSet",             //json任务下发
            "AutoLine"              //自动生成航线上报，未启用
        };

        static const char* topicNames[EDDSTopicType::DDS_TpTypeCount] ={
            "AttackObjectDesignateTopic",//跟踪打击指定目标
			"CommandRequestTopic",		 //命令请求
			"CommandResponseTopic",		 //命令回复
			"ObjectReportSpecialTopic",
			"OpticalPoseTopic",			 //动捕数据，已弃用
			"TaskStageTopic",			 //任务阶段，现用于编队分组及偏移设置
			"SimpleVehicleTopic",		 //精简飞控消息
			"StatusTaskTopic",			 //任务状态
			"VehiWarningTopic",			 //告警消息
			"ObjectReportTopic",
			"StageSetTopic",			 //json任务下发
			"AuotoLineReportTopic"     	 //自动生成航线上报，未启用
        };

		//初始话题qos为BestEffort
        static CDDSPubSub::EQosLvl qosLevels[EDDSTopicType::DDS_TpTypeCount] ={CDDSPubSub::EQosLvl::BestEffort};
        static std::chrono::system_clock::time_point gpreHeartbeatTime;//接收心跳时间
        
        CDDSCtrl::CDDSCtrl(bool isNx):CBaseCtrl(),m_dmp(nullptr)
        ,m_pub(nullptr)
        ,m_sub(nullptr)
        ,m_trd(nullptr)
        ,m_isRun(true)
        ,m_isNx(isNx)
        ,m_hbPrd(8)
        ,m_lstPrd(30){
            m_types[typeNames[DDS_TpAttackObjectDesignate]] = new TypeSupport(new AttackObjectDesignatePubSubType());  
            m_types[typeNames[DDS_TpCommandRequest]] = new TypeSupport(new CommandRequestPubSubType());
            m_types[typeNames[DDS_TpCommandResponse]] = new TypeSupport(new CommandResponsePubSubType());
            m_types[typeNames[DDS_TpObjectReport]] = new TypeSupport(new ObjectReportPubSubType());
            m_types[typeNames[DDS_TpOpticalPose]] = new TypeSupport(new OpticalPosePubSubType());
            m_types[typeNames[DDS_TpTaskStage]] = new TypeSupport(new TaskStagePubSubType());
            m_types[typeNames[DDS_TpSimpleVehicle]] = new TypeSupport(new SimpleVehiclePubSubType());
            m_types[typeNames[DDS_TpStatusTask]] = new TypeSupport(new StatusTaskPubSubType());
            m_types[typeNames[DDS_TpVehiWarning]] = new TypeSupport(new VehiWarningPubSubType());
			m_types[typeNames[DDS_TpSpecialObjectReport]] = new TypeSupport(new ObjectReportPubSubType());
            m_types[typeNames[DDS_TpStageSet]] = new TypeSupport(new StageSetPubSubType());
            m_types[typeNames[DDS_TpAutoLine]] = new TypeSupport(new AutoLinePubSubType());

            qosLevels[EDDSTopicType::DDS_TpCommandRequest] = CDDSPubSub::EQosLvl::Reliable;//设置命令消息qos为可靠
            qosLevels[EDDSTopicType::DDS_TpStageSet] = CDDSPubSub::EQosLvl::Reliable;//设置json任务消息qos为可靠
            qosLevels[EDDSTopicType::DDS_TpAttackObjectDesignate]  = CDDSPubSub::EQosLvl::Reliable;//设置目标跟踪打击消息qos为可靠
            
        }
        CDDSCtrl::~CDDSCtrl(){
            if(m_dmp && m_sub && m_pub){
                for(auto it=m_pubSubs.begin();it != m_pubSubs.end();++it){
                    if(it->second){
                        TypeSupport *type = getType(it->first.c_str());
						//释放相关资源
                        it->second->releaseRes(type,m_dmp,m_sub,m_pub);
                        delete it->second;
                    }            
                  
                }
            }
            
            m_pubSubs.clear();
            for(auto it=m_types.begin();it!=m_types.end();++it){
                delete it->second;
            }
            m_types.clear();
            if(m_sub && m_dmp){
                m_dmp->delete_subscriber(m_sub);
                m_sub = nullptr;
            }
            if(m_pub && m_dmp){
                m_dmp->delete_publisher(m_pub);
                m_pub = nullptr;
            }
            if(m_dmp){
                DomainParticipantFactory::get_instance()->delete_participant(m_dmp);
                m_dmp = nullptr;
            }
        }
        TypeSupport* CDDSCtrl::getType(const char* name){
            auto it = m_types.find(name);
            if(it == m_types.end())
                return nullptr;
            return it->second;
            
        }

        CDDSPubSub* CDDSCtrl::getPubSub(const char* topicName){
            auto it = m_pubSubs.find(topicName);
            if(it == m_pubSubs.end())
                return nullptr;
            return it->second;
        }

        CDDSPubSub* CDDSCtrl::getOptPosePubSub(dm::uint8 id,bool pub,bool sub){
            auto it = m_optPubSubs.find(id);
            if(it == m_optPubSubs.end()){
                TypeSupport* type = getType(typeNames[DDS_TpOpticalPose]);
                if(type && m_dmp){
                    type->register_type(m_dmp);
                    char name[NameLen] = {0};
                    snprintf(name,NameLen,"%s%d",topicNames[DDS_TpOpticalPose],id);
                    CDDSPubSub *pubSub = new CDDSPubSub(name,type,m_dmp,sub?m_sub:nullptr,pub?m_pub:nullptr);
                    m_optPubSubs[id] = pubSub;
                    if(sub)
                        pubSub->sig_topic.connect(boost::bind(&CDDSCtrl::onRcvTopicInfo,this,boost::placeholders::_1));
                    return pubSub;
                }
            }else{
                return it->second;
            }
        }

        CDDSPubSub* CDDSCtrl::getSpeciObjPubSub(dm::uint8 id,bool pub,bool sub){
            auto it = m_objCompPubSubs.find(id);
            if(it == m_objCompPubSubs.end()){
                TypeSupport* type = getType(typeNames[DDS_TpSpecialObjectReport]);
                if(type && m_dmp){
                    type->register_type(m_dmp);
                    char name[NameLen] = {0};
                    snprintf(name,NameLen,"%s%d",topicNames[DDS_TpSpecialObjectReport],id);
                    CDDSPubSub *pubSub = new CDDSPubSub(name,type,m_dmp,sub?m_sub:nullptr,pub?m_pub:nullptr);
                    m_objCompPubSubs[id] = pubSub;
                    if(sub)
                        pubSub->sig_topic.connect(boost::bind(&CDDSCtrl::onRcvTopicInfo,this,boost::placeholders::_1));
                    return pubSub;
                }
            }else{
                return it->second;
            }
        }

         void CDDSCtrl::sendAttactInfo(const CCommApi::SAttackObjectDesignate& info){
            CDDSPubSub* pubSub = getPubSub(topicNames[DDS_TpAttackObjectDesignate]);
            if(pubSub){
                AttackObjectDesignate* attaObj = static_cast<AttackObjectDesignate*>(pubSub->getSendData());
                attaObj->grpid(info.grpid);       //分组id
				attaObj->ids(info.ids);           //分组内飞机id
				attaObj->type(info.type);         //打击方式
                ObjectReport object;
                object.cls(info.objs.cls);
                object.id(info.objs.id);
                object.objid(info.objs.objid);
                object.subcls(info.objs.subcls);
                object.x(info.objs.x);
                object.y(info.objs.y);
                object.z(info.objs.z);
                object.vx(info.objs.vx);
                object.vy(info.objs.vy);
                object.vz(info.objs.vz);
				attaObj->objs(object);           //需要打击目标信息  
                pubSub->sendData();
				char msg[128] = { 0 };
				
            }
         }

        void CDDSCtrl::sendCmdInfo(const CCommApi::SCommandRequest& info){
            CDDSPubSub* pubSub = getPubSub(topicNames[DDS_TpCommandRequest]);
            if(pubSub){
                CommandRequest* cmdReq = static_cast<CommandRequest*>(pubSub->getSendData());
                cmdReq->srcid(info.srcid);
                cmdReq->dstid(info.dstid);
                cmdReq->grpid(info.grpid);
                cmdReq->type(info.type);
                cmdReq->param0(info.param0);
                cmdReq->param1(info.param1);
                cmdReq->param2(info.param2);
                cmdReq->param3(info.param3);
                cmdReq->param4(info.param4);
                cmdReq->fparam5(info.fparam5);
                cmdReq->fparam6(info.fparam6);
                cmdReq->fparam7(info.fparam7);
                cmdReq->fparam8(info.fparam8);
                pubSub->sendData();
				char msg[MsgLen] = { 0 };
                #ifdef __linux
                txtLog().info(THISMODULE  "send cmd type:%d dstid:%d param0:%d param1:%d param2:%d param3:%d param4:%d", info.type, info.dstid,info.param0,
					info.param1,info.param2,info.param3,info.param4);
                #endif
				
            }
        }
     
        bool CDDSCtrl::initByConfig(const char* configName){
          
            return true;
        }

        bool CDDSCtrl::initByDefault(bool isNx){
			m_isNx = isNx;
            init("DDSDomainParticipant");
            if(m_dmp){
                
				//初始全部话题为不发布
                std::vector<bool> pubs(DDS_TpTypeCount,false);
				//初始化全部话题为不订阅
                std::vector<bool> subs(DDS_TpTypeCount,false);
                
                if(isNx){
					//nx机载板卡中订阅 命令请求、设置航线、设置辅助信息、飞机信息，发布 命令回复、设置辅助消息、飞机信息
                    subs[DDS_TpAttackObjectDesignate]  = true;
                    subs[DDS_TpCommandRequest]  = true;
                    subs[DDS_TpTaskStage]  = true;
                    subs[DDS_TpSimpleVehicle]  = true;
					subs[DDS_TpOpticalPose] = true;
					subs[DDS_TpSpecialObjectReport] = true;
                    pubs[DDS_TpCommandResponse]  = true;
                    pubs[DDS_TpObjectReport]  = true;
                    pubs[DDS_TpSimpleVehicle]  = true;
                    pubs[DDS_TpStatusTask]  = true;       
                    pubs[DDS_TpVehiWarning] = true;
					pubs[DDS_TpSpecialObjectReport] = true;
                    subs[DDS_TpStageSet] = true;
                    pubs[DDS_TpAutoLine] = true;          
					                 
				}
				else {//地面站中订阅 请求回复、飞机信息，发布命令请求、设置航线
					subs[DDS_TpSimpleVehicle] = true;
					subs[DDS_TpCommandResponse] = true;
					subs[DDS_TpStatusTask]  = true;
                    subs[DDS_TpVehiWarning] = true;
                    subs[DDS_TpObjectReport] = true;
                    subs[DDS_TpAutoLine] = true;
				
					pubs[DDS_TpCommandRequest] = true;
					pubs[DDS_TpAttackObjectDesignate]  = true;
					pubs[DDS_TpTaskStage]  = true;
					pubs[DDS_TpOpticalPose] = true;
                    pubs[DDS_TpStageSet] = true;
                   
				}
                for(int i=0;i<DDS_TpTypeCount;++i){
                    TypeSupport* type = getType(typeNames[i]);
                    if(type){
                        type->register_type(m_dmp);
                        std::string name(topicNames[i]);
						//依据各话题发布、订阅情况创建发布、订阅对象
                        CDDSPubSub *pubSub = new CDDSPubSub(name,type,m_dmp,subs[i]?m_sub:nullptr,pubs[i]?m_pub:nullptr,qosLevels[i]);
                        m_pubSubs[name] = pubSub;

                        #ifdef __linux
                        txtLog().info(THISMODULE "topicName:%s sub:%d pub:%d",name.c_str(),subs[i]?1:0,pubs[i]?1:0);
                        #endif                     
                    
                        pubSub->sig_topic.connect(boost::bind(&CDDSCtrl::onRcvTopicInfo,this,boost::placeholders::_1));
                    }
                }
                //
                 if(!m_trd){
                    m_trd = std::make_shared<std::thread>(boost::bind(&CDDSCtrl::trdRun,this));
                 }
                return true;
            }
            return false;
        }
    
        void CDDSCtrl::addPubSub(int i,bool pub,bool sub){
            if(i<0 || i>= DDS_TpTypeCount)
                return;
            TypeSupport* type = getType(typeNames[i]);
            if(type){
                type->register_type(m_dmp);
                std::string name(topicNames[i]);
                //依据各话题发布、订阅情况创建发布、订阅对象
                CDDSPubSub *pubSub = new CDDSPubSub(name,type,m_dmp,sub?m_sub:nullptr,pub?m_pub:nullptr);
                m_pubSubs[name] = pubSub;
                if(sub)
                    pubSub->sig_topic.connect(boost::bind(&CDDSCtrl::onRcvTopicInfo,this,boost::placeholders::_1));
            }
        }

        void CDDSCtrl::init(const char* domainName){
            DomainParticipantFactory::get_instance()->load_XML_profiles_file("dds_profiles.xml");
            m_dmp = DomainParticipantFactory::get_instance()->create_participant_with_profile(0,"participant_profile");
            if(m_dmp){
                m_sub = m_dmp->create_subscriber(SUBSCRIBER_QOS_DEFAULT,nullptr);
                m_pub = m_dmp->create_publisher(PUBLISHER_QOS_DEFAULT,nullptr);
                
            }
        }

        void CDDSCtrl::sendSimpleVehicle(const CCommApi::SSimpleVehicle& info){
            CDDSPubSub* pubSub = getPubSub(topicNames[DDS_TpSimpleVehicle]);

            if(pubSub){
                SimpleVehicle* simVehicle = static_cast<SimpleVehicle*>(pubSub->getSendData());
                if((CBaseCtrl::InvalId==m_selfId) && (info.id<CBaseCtrl::MaxId)){
                    #ifdef __linux
                    txtLog().info(THISMODULE "vehicle id:%d",info.id);
                    #endif 
                    setSelfId(info.id);
                }
                simVehicle->time(info.time);
                simVehicle->grp(info.grp);
                simVehicle->id(info.id);
                simVehicle->bat(info.bat);
                simVehicle->fix(info.fix);
                simVehicle->volt(info.volt);
                simVehicle->sate(info.sate);
                simVehicle->lock(info.lock);
                simVehicle->flymd(info.flymd);
                simVehicle->roll(info.roll);
                simVehicle->pitch(info.pitch);
                simVehicle->yaw(info.yaw);
                simVehicle->x(info.x);
                simVehicle->y(info.y);
                simVehicle->z(info.z);
                simVehicle->vx(info.vx);
                simVehicle->vy(info.vy);
                simVehicle->vz(info.vz);
                simVehicle->airspd(info.airspd);
                simVehicle->lon(info.lon);
                simVehicle->lat(info.lat);
                simVehicle->alt(info.alt);
                simVehicle->radz(info.radz);
                pubSub->sendData();

            }
        }

        void  CDDSCtrl::sendTaskStage(const CCommApi::STaskStage& info){
            CDDSPubSub* pubSub = getPubSub(topicNames[DDS_TpTaskStage]);
             
            if(pubSub){
                TaskStage* taskStage = static_cast<TaskStage*>(pubSub->getSendData());
                StageHead stageHead;
                stageHead.grpid(info.head.grpid);          
				stageHead.ids(info.head.ids);    
				stageHead.stage(info.head.stage);            
				stageHead.tag(info.head.tag);     
                taskStage->head(stageHead);       
                taskStage->linx(info.linx);
                taskStage->liny(info.liny);
                taskStage->linz(info.linz);
                taskStage->maxspd(info.maxspd);
                ParamShort paramShort,paramShortform;
                paramShort.type(info.trigger.type);      //参数、指令类型说明
                paramShort.param1(info.trigger.param1 ); //参数1
                paramShort.param2(info.trigger.param2 ); //参数2
                paramShort.param3(info.trigger.param3 ); //参数3
                paramShort.param4(info.trigger.param4 ); //参数4    dm::int32 paramShort.param4(info. ); //参数4
                taskStage->trigger(paramShort);
                paramShortform.type(info.  form.type);    //参数、指令类型说明
                paramShortform.param1(info.form.param1 ); //参数1
                paramShortform.param2(info.form.param2 ); //参数2
                paramShortform.param3(info.form.param3 ); //参数3
                paramShortform.param4(info.form.param4 ); //参数4    dm::int32 paramShort.param4(info. ); //参数4
                taskStage->form(paramShortform);
                taskStage->formOffsetx(info.formOffsetx);
                taskStage->formOffsety(info.formOffsety);
                taskStage->formOffsetz(info.formOffsetz);
                ObjectFilter objectFilter;
                objectFilter.type(info.objs.type);
                objectFilter.path(info.objs.path);
                objectFilter.cls(info.objs.cls);
                objectFilter.subcls(info.objs.subcls);
                taskStage->objs(objectFilter);
                pubSub->sendData();            
            }          
        }

        void CDDSCtrl::sendStatusTask(const CCommApi::SStatusTask& info){
            CDDSPubSub* pubSub = getPubSub(topicNames[DDS_TpStatusTask]);
             
            if(pubSub){
                StatusTask* statusTask = static_cast<StatusTask*>(pubSub->getSendData());
                statusTask->id(info.id);              //飞机id
			    statusTask->stage(info.stage);	      //任务阶段
			    statusTask->tag(info.tag);            //阶段标识 0:前突,1:搜索,2:跟踪打击,3:回收
			    statusTask->status(info.status);      //任务状态
			    statusTask->dstwaypt(info.dstwaypt);  //目标航点id
			    statusTask->diswaypt(info.diswaypt);  //距离目标航点距离
			    statusTask->disobj(info.disobj);      //距离目标距离
			    statusTask->obj(info.obj);            //目标id
                pubSub->sendData();
            }
        }

        void CDDSCtrl::sendVehiWarning(const CCommApi::SVehiWarning& info){
            CDDSPubSub* pubSub = getPubSub(topicNames[DDS_TpVehiWarning]);
             
            if(pubSub){
                VehiWarning* vehiWarning = static_cast<VehiWarning*>(pubSub->getSendData());
                vehiWarning->id(info.id);           //飞机id
			    vehiWarning->src(info.src);	        //任务阶段
			    vehiWarning->tag(info.tag);         //阶段标识 0:前突,1:搜索,2:跟踪打击,3:回收
			    vehiWarning->level(info.level);     //任务状态
			    vehiWarning->desc(info.desc);       //目标航点id
                pubSub->sendData();
            }
        }
 
        void CDDSCtrl::sendResponse(dm::uint8 id,dm::uint16 src,dm::uint16 subType,dm::uint16 status,std::string rslt){
            CDDSPubSub* pubSub = getPubSub(topicNames[DDS_TpCommandResponse]);
            //
              if(pubSub){
                CommandResponse* cmdRes = static_cast<CommandResponse*>(pubSub->getSendData());              
                cmdRes->id(id);
                cmdRes->cmd(src);
                cmdRes->subcmd(subType);
                cmdRes->status(status);
                cmdRes->rslt(rslt);
                pubSub->sendData();
				#ifdef __linux
                txtLog().info(THISMODULE  "send resp id:%d cmd:%d subcmd:%d", id, src, subType);
                #endif
            }
          
        }

        void CDDSCtrl::sendOptPose(dm::uint8 id,const CCommApi::SOpticalPose &info){
            CDDSPubSub* pubSub = getOptPosePubSub(id,true,false);
            if(pubSub){
                OpticalPose *optInfo = static_cast<OpticalPose*>(pubSub->getSendData());
                optInfo->roll(info.roll);
                optInfo->pitch(info.pitch);
                optInfo->yaw(info.yaw);
                optInfo->x(info.x);
                optInfo->y(info.y);
                optInfo->z(info.z);
                pubSub->sendData();
            }
        }

        void CDDSCtrl::sendObjectComputation(const CCommApi::SObjectReport& info){
              CDDSPubSub* pubSub = getPubSub(topicNames[DDS_TpObjectReport]);
            
              if(pubSub){
                ObjectReport* objRep = static_cast<ObjectReport*>(pubSub->getSendData());
                objRep->objid(info.objid);
                objRep->id(m_selfId);
                objRep->cls(info.cls);
                objRep->subcls(info.subcls);
                objRep->score(info.score);
                objRep->x(info.x);
                objRep->y(info.y);
                objRep->z(info.z);
                objRep->vx(info.vx);
                objRep->vy(info.vy);
                objRep->vz(info.vz);
                s_objComputation.insert(info.id);
                pubSub->sendData();
            }
          
        }

        void CDDSCtrl::sendStageSet(const std::string &inf){
            CDDSPubSub *pubSub = getPubSub(topicNames[DDS_TpStageSet]);
            if(pubSub){               
                StageSet *stg = static_cast<StageSet*>(pubSub->getSendData());
                stg->info(inf);
                pubSub->sendData();                           
            }
        }
        
        void CDDSCtrl::sendAutoLine(const CCommApi::SAutoNvaline &info){
            CDDSPubSub *pubSub = getPubSub(topicNames[DDS_TpAutoLine]);
            if(pubSub){
                AutoLine *stg = static_cast<AutoLine*>(pubSub->getSendData());
                stg->id(getSelfId());
                stg->typ(info.type);
                
                auto sz = info.x_lat.size();
                stg->x_lat().reserve(sz);
                stg->y_lon().reserve(sz);
                stg->z_alt().reserve(sz);

                for(int i =0;i<sz;i++)
                {                   
                    stg->x_lat().push_back(info.x_lat[i]);
                    stg->y_lon().push_back(info.y_lon[i]);
                    stg->z_alt().push_back(info.z_alt[i]);
                }                             
                pubSub->sendData();
            }
        }

        void CDDSCtrl::onRcvTopicInfo(CDDSPubSub* pubSub){
            const std::string & tpName = pubSub->getTopic()->get_name();
            
            if (tpName == topicNames[DDS_TpAttackObjectDesignate])
            {
                processRcvAttack(pubSub->getRcvData());   

            }else if(tpName == topicNames[DDS_TpCommandRequest])
            {
                processRcvCmdReq(pubSub->getRcvData());

            }else if(tpName == topicNames[DDS_TpCommandResponse]){

                processRcvCmdRes(pubSub->getRcvData());

            }else if(tpName == topicNames[DDS_TpSimpleVehicle]){

                processRcvVehicle(pubSub->getRcvData());

            }else if(tpName.find(topicNames[DDS_TpOpticalPose])==0){
                
                processRcvOptPose(pubSub->getRcvData());
            }else if(tpName == topicNames[DDS_TpTaskStage]){
                processRcvTaskStage(pubSub->getRcvData());
            }else if(tpName == topicNames[DDS_TpObjectReport]){

                processRcvObjectReport(pubSub->getRcvData());
            }else if(tpName == topicNames[DDS_TpStageSet]){
                processRcvStageSetJson(pubSub->getRcvData());

            }else if(tpName == topicNames[DDS_TpAutoLine]){
                processRcvAutoLine(pubSub->getRcvData());
            }else if(tpName == topicNames[DDS_TpStatusTask]){
                processRcvStatusTask(pubSub->getRcvData());
            }           
        }

        void CDDSCtrl::setSelfId(dm::uint8 id){
            m_selfId = id;
            sendResponse(m_selfId,0,CCommApi::ECmd::Test,0,"test info");
        }

        void CDDSCtrl::processRcvAttack(void* data){
            AttackObjectDesignate* info = static_cast<AttackObjectDesignate*>(data);
            m_attackInfo.grpid = info->grpid();
            m_attackInfo.ids = info->ids();
            m_attackInfo.type = info->type();
            m_attackInfo.objs.cls=info->objs().cls();
            m_attackInfo.objs.id=info->objs().id();
            m_attackInfo.objs.objid=info->objs().objid();
            m_attackInfo.objs.subcls=info->objs().subcls();
            m_attackInfo.objs.vx=info->objs().vx();
            m_attackInfo.objs.vy=info->objs().vy();
            m_attackInfo.objs.vz=info->objs().vz();
            m_attackInfo.objs.x=info->objs().x();
            m_attackInfo.objs.y=info->objs().y();
            m_attackInfo.objs.z=info->objs().z();
            sig_rcvAtact(m_attackInfo); 
			#ifdef __linux
            txtLog().info(THISMODULE  "rcv Attack or trace type:%d x:%d y:%d z:%d", m_attackInfo.type, m_attackInfo.objs.x, m_attackInfo.objs.y, m_attackInfo.objs.z);
			#endif

        }
       
        void CDDSCtrl::processRcvStatusTask(void* data){             
            StatusTask* info = static_cast<StatusTask*>(data);
            m_statusTask.id=info->id();          //飞机id
            m_statusTask.stage=info->stage();	   //任务阶段
            m_statusTask.tag=info->tag();         //阶段标识 0:前突,1:搜索,2:跟踪打击,3:回收
            m_statusTask.status=info->status();      //任务状态
            m_statusTask.dstwaypt=info->dstwaypt();   //目标航点id
            m_statusTask.diswaypt=info->diswaypt();   //距离目标航点距离
            m_statusTask.disobj=info->disobj();     //距离目标距离
            m_statusTask.obj=info->obj();        //目标id
            sig_rcvStatusTask(m_statusTask);
        }

        void CDDSCtrl::processRcvVehiWarning(void* data){            
            VehiWarning* info = static_cast<VehiWarning*>(data);     
			if (info) {
				CCommApi::SVehiWarning warn;
				warn.id = info->id();
				warn.src = info->src();
				warn.tag = info->tag();
				warn.level = info->level();
				warn.desc = info->desc();
				sig_rcvWarning(warn);
			}
			
        }

        void CDDSCtrl::processRcvCmdReq(void* data){
            CommandRequest* info = static_cast<CommandRequest*>(data);
			static bool sendPong = false;
			//为测试指令时直接回复，不将其转发为ros消息
			if (info->type() == CCommApi::ECmd::Test) {
				sendResponse(m_selfId,0,info->type(),0,"rcv test info success");
                #ifdef __linux
                txtLog().info(THISMODULE "rcv test and dont process");
                #endif
				return;
			}else if(CCommApi::ECmd::SendHeartBeat==info->type()){//update heartbeat time
				gpreHeartbeatTime = std::chrono::system_clock::now();
				if (!sendPong) {
                    int64_t nt = (std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::system_clock::now().time_since_epoch())).count();
                    char buf[64]={0};
                    sprintf(buf,"%lld",nt);
					sendResponse(m_selfId, 0, CCommApi::ECmd::Ping, 0, buf);
					sendPong = true;
					#ifdef __linux
                    txtLog().info(THISMODULE  "rcv heartbeat and send ping response(pong)");
                    #endif					
				}

                return;
            }
            
            m_cmdInfos[info->dstid()].srcid=info->srcid();
            m_cmdInfos[info->dstid()].dstid=info->dstid();
            m_cmdInfos[info->dstid()].grpid=info->grpid();
            m_cmdInfos[info->dstid()].type=(CCommApi::ECmd)info->type();
            m_cmdInfos[info->dstid()].param0=info->param0();
            m_cmdInfos[info->dstid()].param1=info->param1();
            m_cmdInfos[info->dstid()].param2=info->param2();
            m_cmdInfos[info->dstid()].param3=info->param3();
            m_cmdInfos[info->dstid()].param4=info->param4();
            m_cmdInfos[info->dstid()].fparam5=info->fparam5();
            m_cmdInfos[info->dstid()].fparam6=info->fparam6();
            m_cmdInfos[info->dstid()].fparam7=info->fparam7();
            m_cmdInfos[info->dstid()].fparam8=info->fparam8();
            sig_rcvCmd(m_cmdInfos[info->dstid()]);
            #ifdef __linux
            txtLog().info(THISMODULE "rcv cmd %d from %d dstId:%d",info->type(),info->srcid(),info->dstid());
            #endif
        }

        void CDDSCtrl::processRcvCmdRes(void* data){
            CommandResponse* info = static_cast<CommandResponse*>(data);
			if (0 == info->cmd() && CCommApi::ECmd::Ping == info->subcmd()) {
				//同步记载发来的时间戳
                #ifndef __linux
                
                if(info->rslt().length() == 0){
                    return;
                }
                
                time_t rt = std::atoll(info->rslt().c_str());
                unsigned short ms = rt%1000;//毫秒部分
                rt /= 1000;//毫秒转秒
                tm * t = localtime(&rt);
				
                SYSTEMTIME st = {};
                st.wYear = t->tm_year + 1900; // tm的年份是从1900年开始的偏移量
                st.wMonth = t->tm_mon + 1;    // tm的月份是0-11
                st.wDay = t->tm_mday;
                st.wHour = t->tm_hour;
                st.wMinute = t->tm_min;
                st.wSecond = t->tm_sec;
                st.wDayOfWeek = t->tm_wday;
                st.wMilliseconds = ms; // 毫秒
                SetLocalTime(&st);//设置本地时间
                
                #endif
				return;
			}
			sig_rcvResp(info->id(), info->cmd(), info->subcmd(), info->status(), info->rslt());
			//收到命令回复消息时
        }

        void CDDSCtrl::processRcvAuxi(void* data){
            
            CommandRequest* info = static_cast<CommandRequest*>(data);
            if(info->srcid() != m_selfId){
                m_auxiInfos[info->dstid()].srcid=info->srcid();
                m_auxiInfos[info->dstid()].dstid=info->dstid();
                m_auxiInfos[info->dstid()].grpid=info->grpid();
                m_auxiInfos[info->dstid()].type=(CCommApi::ECmd)info->type();
                m_auxiInfos[info->dstid()].param0=info->param0();
                m_auxiInfos[info->dstid()].param1=info->param1();
                m_auxiInfos[info->dstid()].param2=info->param2();
                m_auxiInfos[info->dstid()].param3=info->param3();
                m_auxiInfos[info->dstid()].param4=info->param4();
                m_auxiInfos[info->dstid()].fparam5=info->fparam5();
                m_auxiInfos[info->dstid()].fparam6=info->fparam6();
                m_auxiInfos[info->dstid()].fparam7=info->fparam7();
                m_auxiInfos[info->dstid()].fparam8=info->fparam8();
				//触发接收到辅助消息信号
                sig_rcvAuxi(m_auxiInfos[info->dstid()]);
            }
            
        }

        void CDDSCtrl::processRcvVehicle(void* data){
            SimpleVehicle* info = static_cast<SimpleVehicle*>(data);
 
            if(info->id() != m_selfId){
                m_vehicleSimples[info->id()].time=info->time();
                m_vehicleSimples[info->id()].grp=info->grp();
                m_vehicleSimples[info->id()].id=info->id();
                m_vehicleSimples[info->id()].bat=info->bat();
                m_vehicleSimples[info->id()].fix=info->fix();
                m_vehicleSimples[info->id()].volt=info->volt();
                m_vehicleSimples[info->id()].sate=info->sate();
                m_vehicleSimples[info->id()].lock=info->lock();
                m_vehicleSimples[info->id()].flymd=info->flymd();
                m_vehicleSimples[info->id()].roll=info->roll();
                m_vehicleSimples[info->id()].pitch=info->pitch();
                m_vehicleSimples[info->id()].yaw=info->yaw();
                m_vehicleSimples[info->id()].x=info->x();
                m_vehicleSimples[info->id()].y=info->y();
                m_vehicleSimples[info->id()].z=info->z();
                m_vehicleSimples[info->id()].vx=info->vx();
                m_vehicleSimples[info->id()].vy=info->vy();
                m_vehicleSimples[info->id()].vz=info->vz();
                m_vehicleSimples[info->id()].airspd=info->airspd();
                m_vehicleSimples[info->id()].lon=info->lon();
                m_vehicleSimples[info->id()].lat=info->lat();
                m_vehicleSimples[info->id()].alt=info->alt();
                m_vehicleSimples[info->id()].radz=info->radz();
    
				//触发接收到飞机信息信号
                sig_rcvVehicleSimple(m_vehicleSimples[info->id()]);
            }            
        }

        void CDDSCtrl::processRcvTaskStage(void* data){
            TaskStage* info = static_cast<TaskStage*>(data);
            m_taskStage.head.grpid=info->head().grpid();
            m_taskStage.head.ids=info->head().ids();
            m_taskStage.head.stage=info->head().stage();
            m_taskStage.head.tag=info->head().tag();
            m_taskStage.linx=info->linx();
            m_taskStage.liny=info->liny();
            m_taskStage.linz=info->linz();
            m_taskStage.maxspd=info->maxspd();
            m_taskStage.objs.cls=info->objs().cls();
            m_taskStage.objs.path=info->objs().path();
            m_taskStage.objs.subcls=info->objs().subcls();
            m_taskStage.objs.type=info->objs().type();
            m_taskStage.scopetype=info->scopetype();
            m_taskStage.trigger.type=info->trigger().type();
            m_taskStage.trigger.param1=info->trigger().param1();
            m_taskStage.trigger.param2=info->trigger().param2();
            m_taskStage.trigger.param3=info->trigger().param3();
            m_taskStage.trigger.param4=info->trigger().param4();
            m_taskStage.form.type=info->trigger().type();
            m_taskStage.form.param1=info->form().param1();
            m_taskStage.form.param2=info->form().param2();
            m_taskStage.form.param3=info->form().param3();
            m_taskStage.form.param4=info->form().param4();
            m_taskStage.formOffsetx=info->formOffsetx();
            m_taskStage.formOffsety=info->formOffsety();
            m_taskStage.formOffsetz=info->formOffsetz();
            m_taskStage.objs.cls=info->objs().type();
            m_taskStage.objs.path=info->objs().path();
            m_taskStage.objs.subcls=info->objs().subcls();
            m_taskStage.objs.type=info->objs().type();

            sig_rcvTaskStage(m_taskStage);

        }

        void CDDSCtrl::processRcvObjectReport(void* data){
             
            ObjectReport* info = static_cast<ObjectReport*>(data);
            m_objectReport.cls=info->cls();       
            m_objectReport.id=info->id();
            m_objectReport.objid=info->objid();
            m_objectReport.subcls=info->subcls();
            m_objectReport.score=info->score();
            m_objectReport.vx=info->vx();
            m_objectReport.vy=info->vy();
            m_objectReport.vz=info->vz();
            m_objectReport.x=info->x();
            m_objectReport.y=info->y();
            m_objectReport.z=info->z();			
            sig_rcvObjComputation(m_objectReport);
     
        }

        void CDDSCtrl::processSpecifalObjectComputation(dm::uint id,void* info){
            AttackObjectDesignate* atacdinfo = static_cast<AttackObjectDesignate*>(info);
            auto it = s_objComputation.find(id);
            if(it!=s_objComputation.end()){
                CDDSPubSub* pubSub = getSpeciObjPubSub(*it,true,false);//get pub  pub dds and transform outer message
                if(pubSub){
                ObjectReport *objinfo = static_cast<ObjectReport*>(pubSub->getSendData());
                objinfo->id(atacdinfo->objs().id());
                objinfo->objid(atacdinfo->objs().objid());
                objinfo->cls(atacdinfo->objs().cls());
                objinfo->subcls(atacdinfo->objs().subcls());
				objinfo->score(atacdinfo->objs().score());
                objinfo->x(atacdinfo->objs().x());
                objinfo->y(atacdinfo->objs().y());
                objinfo->z(atacdinfo->objs().z());
                objinfo->vx(atacdinfo->objs().vx());
                objinfo->vy(atacdinfo->objs().vy());
                objinfo->vz(atacdinfo->objs().vz());
                pubSub->sendData();

                }
            else{
                CDDSPubSub* pubSub = getSpeciObjPubSub(*it,false,true);//get sub rec dds
                }
            }
        }

		void CDDSCtrl::processRcvStageSetJson(void *data) {
			StageSet *inf = static_cast<StageSet *>(data);
					
			sig_rcvStageSet(inf->info());
            sendResponse(m_selfId,ETopCmd::SendJsonTask,0,ERespStatus::RespRcvWillDo,"rcv json task just will do");
			#ifdef __linux
            txtLog().info(THISMODULE inf->info().c_str());
            #endif			

		}
        
        void CDDSCtrl::processRcvAutoLine(void *data){
            AutoLine *inf = static_cast<AutoLine *>(data);
            
            if(inf){
                sig_lineRpt(inf->id(),inf->typ(),inf->x_lat(),inf->y_lon(),inf->z_alt());
            }
        }

        void CDDSCtrl::processRcvOptPose(void* data){
            OpticalPose* info = static_cast<OpticalPose*>(data);

            m_optPoses[m_selfId].x = info->x();
            m_optPoses[m_selfId].y = info->y();
            m_optPoses[m_selfId].z = info->z();
            m_optPoses[m_selfId].roll = info->roll();
            m_optPoses[m_selfId].pitch = info->pitch();
            m_optPoses[m_selfId].yaw = info->yaw();
            sig_rcvOptPose(m_selfId,m_optPoses[m_selfId]);
        }

        void CDDSCtrl::trdRun(){
            CCommApi::SCommandRequest cmdReq;
            cmdReq.type = CCommApi::ECmd::SendHeartBeat;
            cmdReq.dstid = 0;
            std::chrono::nanoseconds hb(m_hbPrd*1000000000);//heart beat period (
            std::chrono::nanoseconds lst(m_lstPrd*1000000000);//lost commu time (s)
            std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
      
            #ifdef __linux
            txtLog().info(THISMODULE "start");
            #endif
            gpreHeartbeatTime = std::chrono::system_clock::time_point();//

            while(m_isRun){
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                tp = std::chrono::system_clock::now();


                if(m_isNx){//check if lost heartbeat
                    if(gpreHeartbeatTime.time_since_epoch().count()>0 && ((tp-gpreHeartbeatTime) >= lst)){
                        cmdReq.type = CCommApi::ECmd::Return;
                        cmdReq.dstid = m_selfId;
                        sig_rcvCmd(cmdReq);
                        gpreHeartbeatTime = std::chrono::system_clock::time_point();//

                    }
                }else{//send heartbeat
                    if((tp-gpreHeartbeatTime) >= hb){//send heartbeat                   
                       sendCmdInfo(cmdReq);
                       gpreHeartbeatTime = tp;
                    }
                }
            }
        }

    } // namespace commu
    
    
} // namespace zyzn

