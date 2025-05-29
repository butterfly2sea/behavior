#ifdef _MSC_VER
#define COMMU_API __declspec(dllexport)
#endif
#include"CommApi.h"
#include"DDSCtrl.h"


namespace zyzn
{
    
    namespace commu
    {
        CBaseCtrl* CCommApi::m_commuCtrl = nullptr;
        int CCommApi::m_refCount = 0;
        CCommApi::CCommApi(f_vehiCB_t vehicleSipleCB,f_respCB_t cmdResCB,f_warnCB_t warnCB,f_taskStatusCB_t taskCB, f_objCB_t objCB):m_vehiCB(vehicleSipleCB),
        m_respCB(cmdResCB),
        m_warnCB(warnCB),
        m_taskStatusCB(taskCB),
		m_objCB(objCB),
	    m_lineCB(nullptr)
        {
			m_mtx.lock();
			memset(&m_cmdInfo, 0, sizeof(SCommandRequest));
            if(nullptr == m_commuCtrl){
                m_commuCtrl = new CDDSCtrl(false);
                m_commuCtrl->initByDefault(false);
                ++m_refCount;
            }
            if(m_commuCtrl){
                //m_commuCtrl->sig_rcvVehicleSimple.connect( boost::bind(&CCommApi::onRcvVehicleSimple,this,boost::placeholders::_1) );
                //m_commuCtrl->sig_rcvResp.connect(boost::bind(&CCommApi::onRcvResp,this,boost::placeholders::_1,boost::placeholders::_2,boost::placeholders::_3,boost::placeholders::_4,boost::placeholders::_5));
                if(m_vehiCB) 
                    m_commuCtrl->sig_rcvVehicleSimple.connect(m_vehiCB);
                if(m_respCB)
                    m_commuCtrl->sig_rcvResp.connect(m_respCB);
				if (m_warnCB)
					m_commuCtrl->sig_rcvWarning.connect(std::bind(&CCommApi::onRcvWarn,this,std::placeholders::_1));
				if (m_taskStatusCB)
					m_commuCtrl->sig_rcvStatusTask.connect(std::bind(&CCommApi::onRcvTaskStatus,this,std::placeholders::_1));
				if(m_objCB)
					m_commuCtrl->sig_rcvObjComputation.connect(std::bind(&CCommApi::onRcvObj,this,std::placeholders::_1));
				
            } 
			m_mtx.unlock();

        }
        CCommApi::~CCommApi(){
            m_refCount--;
            if(m_refCount <=0 && m_commuCtrl){
                delete m_commuCtrl;
                m_commuCtrl = nullptr;
            }
        }
        void CCommApi::commuTest(dm::uint8 dstid,dm::uint8 grpId,dm::uint8 srcId){
            if(m_commuCtrl){
                m_cmdInfo.dstid = dstid;
                m_cmdInfo.srcid = srcId;
                m_cmdInfo.grpid = grpId;
                m_cmdInfo.type = Test;
               
                m_commuCtrl->sendCmdInfo(m_cmdInfo);
            }
            
        }
        void CCommApi::sendCommonInf(const SCommandRequest &inf) {

			if (m_commuCtrl) {
				m_commuCtrl->sendCmdInfo(inf);
			}
		}

        void CCommApi::sendTaskStage(const STaskStage &inf){
			if (m_commuCtrl) {
				m_commuCtrl->sendTaskStage(inf);
			}
        }

		void CCommApi::sendAttackObj(const SAttackObjectDesignate &inf){
			if (m_commuCtrl) {
				m_commuCtrl->sendAttactInfo(inf);
			}
        }

		void CCommApi::sendOptical(dm::uint8 id,const SOpticalPose &inf){
			if (m_commuCtrl) {
				m_commuCtrl->sendOptPose(id, inf);
			}
        }

		void CCommApi::sendJsonStage(const std::string &info) {
			if (m_commuCtrl) {			
				m_commuCtrl->sendStageSet(info);				
			}

		}
		
		void CCommApi::setAutoLineCB(f_autoLineCB_t autoLineCB) {
			if(autoLineCB && m_commuCtrl){
                m_commuCtrl->sig_lineRpt.connect(autoLineCB);
            }
		}


		void CCommApi::onRcvObj(const CCommApi::SObjectReport &info) {
			dm::uint32 spd = sqrt(info.vx*info.vx + info.vy*info.vy + info.vz*info.vz);
			m_objCB(info.id, info.objid, info.cls, info.subcls, info.score,info.x, info.y, info.z, spd);
		}
		void CCommApi::onRcvWarn(const CCommApi::SVehiWarning &info) {
			m_warnCB(info.id, info.src, info.tag, info.level, info.desc);
		}
		void CCommApi::onRcvTaskStatus(const CCommApi::SStatusTask& info) {
			m_taskStatusCB(info.id, info.stage, info.tag,info.status, info.dstwaypt, info.diswaypt, info.disobj, info.obj);
		}
        
    } // namespace commu}
    
} // namespace zyzn
