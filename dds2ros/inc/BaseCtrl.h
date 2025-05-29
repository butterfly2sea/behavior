#ifndef BASECTRL_ZYZN_COMMU_H
#define BASECTRL_ZYZN_COMMU_H

#include"CommApi.h"
#include <boost/smart_ptr/scoped_array.hpp>
#include <boost/signals2/signal.hpp>

/*
* 通信控制基类，用于控制的发送及数据获取DDS控制类及ROS控制类均继承它
*/
namespace zyzn
{
    namespace commu
    {
		/*
		* @brief ros话题类型
		*/
         enum ETopicType{
            TpAttackObjectDesignate=0,  ///<! 跟踪打击指定目标>             
            TpCommandRequest,           ///<! 命令请求>               
            TpCommandResponse,          ///<! 命令回复>             
            TpObjectReport,             ///<! 目标上报>              
            TpOpticalPose,              ///<! 动捕数据>             
            TpTaskStage,                ///<! 编队分组及偏移设置>               
            TpSimpleVehicle,            ///<! 其它飞机精简飞控消息>        
            TpStatusTask,               ///<! 任务执行状态>        
            TpVehiWarning,              ///<! 告警消息>            
            TpInnerSimpleVehicle,       ///<! 本机精简飞控消息>          
            TpSpecialObjReport,         ///<! 用于它机目标报告> 
            TpStageSet,                 ///<! json任务下发> 
            TpAutoLine,                 ///<! 自动生成航线上报> 
            TpTypeCount  

        };

        /**
         * @brief DDS话题类型
         */
        enum EDDSTopicType{
            DDS_TpAttackObjectDesignate=0,///<! 跟踪打击指定目标> 
            DDS_TpCommandRequest,         ///<! 命令请求>
            DDS_TpCommandResponse,        ///<! 命令回复>
            DDS_TpObjectReport,           ///<! 目标上报>
            DDS_TpOpticalPose,            ///<! 动捕数据>
            DDS_TpTaskStage,              ///<! 编队分组及偏移设置>
            DDS_TpSimpleVehicle,          ///<! 飞机精简飞控消息>
            DDS_TpStatusTask,             ///<! 任务执行状态>
            DDS_TpVehiWarning,            ///<! 告警消息>
            DDS_TpSpecialObjectReport,    ///<! 用于它机目标报告>
            DDS_TpStageSet,               ///<! json任务下发>
            DDS_TpAutoLine,               ///<! 自动生成航线上报>
            DDS_TpTypeCount
            
        };

        enum{
            NumLen=10,    ///<! 编号长度>
            NameLen=50,   ///<! 话题名称长度>
			MsgLen = 128, ///<! 消息长度>
			PathLen = 256 ///<! 路径长度>
        };

        /**
         * @brief 指令回复中顶层命令类型
         */
        enum ETopCmd{
            CmdRequest=0, ///<! 命令请求>
            ObjAtckTrack, ///<! 跟踪打击指定目标>
            SendJsonTask ///<! json任务下发>
        };

        /**
         * @brief 指令回复中状态
         */
        enum ERespStatus{
            RespSuccess=0, ///<! 指令执行成功>
            RespFailed,    ///<! 指令执行失败>
            RespRcvWillDo  ///<! 指令已接收，待执行>
        };



		
        class CBaseCtrl
        {
        public:

        enum {
            InvalId = 0,
            MaxId = 200
        };
       
        typedef void (f_attact_t)(const CCommApi::SAttackObjectDesignate&);
        typedef void (f_cmd_t) (const CCommApi::SCommandRequest&);
        typedef void (f_vehi_t) (const CCommApi::SSimpleVehicle&);
        typedef void (f_warning_t) (const CCommApi::SVehiWarning&);
        typedef void (f_taskStage_t) (const CCommApi::STaskStage&);
        typedef void (f_statusTask_t) (const CCommApi::SStatusTask&);
        typedef void (f_objComp_t) (const CCommApi::SObjectReport&);
        typedef void (f_autonva_t) (const CCommApi::SAutoNvaline&);

        typedef void (f_resp_t) (dm::uint8,dm::uint8,dm::uint16,dm::uint16,std::string);
		typedef void (f_optPose_t) (dm::uint8,const CCommApi::SOpticalPose &);
        typedef void (f_stageset_t) (const std::string &);
		typedef void (f_lineRpt_t)(dm::uint8, dm::uint8, std::vector<int>&, std::vector<int>&, std::vector<int>&);

        typedef boost::signals2::signal<f_attact_t> sig_attact_t;
        typedef boost::signals2::signal<f_cmd_t> sig_cmd_t;
        typedef boost::signals2::signal<f_vehi_t> sig_vehi_t;
        typedef boost::signals2::signal<f_warning_t> sig_warning_t;
        typedef boost::signals2::signal<f_autonva_t> sig_autonva_t;
		typedef boost::signals2::signal<f_lineRpt_t> sig_lineRpt_t;

        typedef boost::signals2::signal<f_resp_t> sig_resp_t;
		typedef boost::signals2::signal<f_optPose_t> sig_optPose_t;
        typedef boost::signals2::signal<f_statusTask_t> sig_statusTask_t;
        typedef boost::signals2::signal<f_taskStage_t> sig_taskStage_t;
        typedef boost::signals2::signal<f_objComp_t> sig_objComp_t;
        typedef boost::signals2::signal<f_stageset_t> sig_stageSet_t;

		
        sig_attact_t sig_rcvAtact;       //接收到attact命令时触发的信号
        sig_cmd_t sig_rcvCmd;            //接收到控制命令时触发的信号
        sig_resp_t sig_rcvResp;          //接收到回复信息时触发的信号
        sig_cmd_t sig_rcvAuxi;           //接收到辅助信息时触发的信号
        sig_vehi_t sig_rcvVehicleSimple; //接收到简单飞控信息时触发的信号
        sig_warning_t sig_rcvWarning;    //接收到告警信息时触发的信号
        sig_taskStage_t sig_rcvTaskStage;//接收到结构体阶段信息时触发的信号
        sig_statusTask_t sig_rcvStatusTask;//接收到任务状态时触发的信号
        sig_objComp_t sig_rcvObjComputation;//接收到

		sig_optPose_t sig_rcvOptPose;       //接收到光学数据触发的信号
        sig_stageSet_t sig_rcvStageSet;     //接收到json阶段信息时触发的信号
        sig_autonva_t sig_rcvAutoNva;
		sig_lineRpt_t sig_lineRpt;
        
		
        
        private:
            /* data */
        public:
            CBaseCtrl(/* args */){m_selfId=0;}
            virtual ~CBaseCtrl(){}
			/*
			* 发送控制信息
			* info 控制信息内容
            */
            virtual void sendCmdInfo(const CCommApi::SCommandRequest& info){}


			/*
			* 依据xml配置文件对通信控制进行初始化
			* configName 配置文件名
			*/
            virtual bool initByConfig(const char* configName){
                return false;
            }

			/*
			* 使用默认方式对通信控制进行初始化，和使用配置文件初始化互斥
			*/
			virtual bool initByDefault(bool isNx = true) { return false; }

			/*
			* 发送简单飞控信息
			* info 飞控信息内容
			*/
            virtual void sendVehicleSimple(const CCommApi::SSimpleVehicle& info){}

			/*
			* 发送辅助信息
			* info 辅助信息内容
			*/
            virtual void sendAuxiInfo(const CCommApi::SCommandRequest& info){}

			/*
			* 发送回复信息
			* id 回复节点id
			* type 控制命令类型
			* status 命令执行状态
			* rslt 结果提示信息
			*/
            virtual void sendResponse(dm::uint8 id,dm::uint16 type,dm::uint16 subType,dm::uint16 status,std::string rslt){}
			
			/**
			*发送光学定位信息
			*info 定位信息
			*/
			virtual void sendOptPose(dm::uint8 id,const CCommApi::SOpticalPose &info){}
			
			/**
			*发送任务阶段信息
			*info 阶段信息
			*/
			virtual void sendTaskStage(const CCommApi::STaskStage& info){}
			
			/**
			*发送任务状态信息
			*info 状态信息
			*/
            virtual void sendStatusTask(const CCommApi::SStatusTask& info){}
            //void sendAuxiInfo(const CCommApi::SCommandRequest& info);
			
			/**
			*发送上报目标信息
			*info 目标信息
			*/
            virtual void sendObjectComputation(const CCommApi::SObjectReport& info){}
			
			/**
			*发送指定跟踪、打击目标信息
			*info 目标信息
			*/
			virtual void sendAttactInfo(const CCommApi::SAttackObjectDesignate& info){}

            /**
            *发送阶段设置json信息
            *info 阶段信息
            */
            virtual void sendStageSet(const std::string &info){}


			//获取辅助信息
            inline std::map<dm::uint8,CCommApi::SCommandRequest> & auxiInfos(){
                return m_auxiInfos;
            }

			//获取简单飞控信息
            inline std::map<dm::uint8,CCommApi::SSimpleVehicle> & vehicleSimples(){
                return m_vehicleSimples;
            }

			//设置节点id
            virtual void setSelfId(dm::uint8 id){
                m_selfId = id;
            }

			//获取节点id
            virtual dm::uint8 getSelfId(){
                return m_selfId;
            }
			inline std::map<dm::uint8,CCommApi::SOpticalPose> & optPoses(){
                return m_optPoses;
            }
            
            protected:
            std::map<dm::uint8,CCommApi::SCommandRequest> m_cmdInfos;           //控制命令信息key：目标飞机id
            std::map<dm::uint8,CCommApi::SSimpleVehicle> m_vehicleSimples;//简单飞控信息key：飞机id
            std::map<dm::uint8,CCommApi::SCommandRequest> m_auxiInfos;           //辅助信息key：飞机id
            dm::uint8 m_selfId;                                           //本节点id
			std::map<dm::uint8,CCommApi::SOpticalPose> m_optPoses;
            std::set<dm::uint8> s_objComputation;
        };
        

        
        
    } // namespace commu
    
    
} // namespace zyzn


#endif