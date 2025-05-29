#ifndef DDSCTRL_ZYZN_COMMU_H
#define DDSCTRL_ZYZN_COMMU_H

#include <thread>
#include"BaseCtrl.h"
#include"DDSPubSub.h"
#include<stdio.h>

/*
	DDS通信控制类，用于接收及发送DDS消息
	接收到DDS消息后触发对应的信号，CCommApi绑定对应信号并调用相关回调函数，CRosCtrl绑定对应信号并将数据转换为ros消息进行发布
*/

namespace zyzn
{
    namespace commu
    {
        class CDDSCtrl : public CBaseCtrl{
            public:
            CDDSCtrl(bool isNx=true);
            ~CDDSCtrl();
            bool initByConfig(const char* configName);
            bool initByDefault(bool isNx=true);
            void sendAttactInfo(const CCommApi::SAttackObjectDesignate& info);
            void sendCmdInfo(const CCommApi::SCommandRequest& info);
            void sendSimpleVehicle(const CCommApi::SSimpleVehicle& info);
            void sendTaskStage(const CCommApi::STaskStage& info);
            void sendStatusTask(const CCommApi::SStatusTask& info);
            void sendVehiWarning(const CCommApi::SVehiWarning& info);
            void sendResponse(dm::uint8 id,dm::uint16 type,dm::uint16 subType,dm::uint16 status,std::string rslt);
            void sendOptPose(dm::uint8 id,const CCommApi::SOpticalPose &info);
            void sendObjectComputation(const CCommApi::SObjectReport& info);
            void setSelfId(dm::uint8 id);
            void sendStageSet(const std::string &info);
            void sendAutoLine(const CCommApi::SAutoNvaline &info);

			//接收到主题消息为CDDSPubSub的sig_topic信号的绑定函数
            void onRcvTopicInfo(CDDSPubSub* pubSub);
            
            private:
            void init(const char* domainName);
            void addPubSub(int tpIdx,bool pub=true,bool sub=true);
			//依据类型名获取类型
            TypeSupport* getType(const char* name); 
			//依据话题名获取发布订阅对象
            CDDSPubSub* getPubSub(const char* topicName);
            //处理接收的控制命令
            void processRcvAttack(void* data);
			//处理接收的控制命令
            void processRcvCmdReq(void* data);
			//处理接收的回复信息
            void processRcvCmdRes(void* data);
			//处理接收的辅助信息
            void processRcvAuxi(void* data);
			//处理接收的简单飞控信息
            void processRcvVehicle(void* data);
            //处理接收的简单飞控信息
            void processRcvTaskStage(void* data);
            //处理接收的简单飞控信息
            void processRcvStatusTask(void* data);
            //处理接收的简单飞控信息
            void processRcvVehiWarning(void* data);
			//处理接收到的航线及区域信息
            void processRcvLineArea(void* data);
            //处理接收的简单飞控信息
            void processRcvObjectReport(void* data);
            //处理接收的简单飞控信息
            void processRcvOptPose(void* data);
            //special iobject information 
            void processSpecifalObjectComputation(dm::uint id,void* info);
            //
            //special iobject information 
            void processRcvStageSetJson(void *data);
            void processRcvAutoLine(void *data);

            CDDSPubSub* getOptPosePubSub(dm::uint8 id,bool pub=true,bool sub=true);
            CDDSPubSub* getSpeciObjPubSub(dm::uint8 id,bool pub,bool sub);

            void trdRun();
            
            
            std::map<std::string,TypeSupport*> m_types; //全部类型key：类型名
            std::map<std::string,CDDSPubSub*> m_pubSubs;//全部发布订阅对象，key：话题名
            std::map<dm::uint8,CDDSPubSub*> m_optPubSubs;//
            std::map<dm::uint8,CDDSPubSub*> m_objCompPubSubs;//
            DomainParticipant* m_dmp;
            Subscriber* m_sub;
            Publisher* m_pub;

            CCommApi::SAttackObjectDesignate m_attackInfo;
            CCommApi::SStatusTask m_statusTask;
            CCommApi::SVehiWarning m_vehiWarning; 
            CCommApi::STaskStage m_taskStage;  
            CCommApi::SObjectReport m_objectReport; 
            CCommApi::SAutoNvaline m_autoNvaline; 

            std::string m_stageSet;
			int m_partCount;
            std::shared_ptr<std::thread> m_trd;
            bool m_isRun;
            bool m_isNx;
            int64_t  m_hbPrd;    //心跳周期 秒
            int64_t  m_lstPrd;   //失联时长 秒

        };
    } // namespace commu
    
    
} // namespace zyzn

#endif