#ifndef COMMAPI_ZYZN_H
#define COMMAPI_ZYZN_H


#include<vector>
#include<string>
#include<set>
#include<functional>
#include<mutex>
#include<dm/types.hpp>
/*
	通信接口类，用于地面站发送控制命令及接收飞机信息
*/
#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4530 )
#pragma warning( disable : 4786 )
#else
#define COMMU_API
#endif


#ifndef COMMU_API
#define COMMU_API __declspec(dllimport)
#endif


namespace zyzn
{
    namespace commu
    {
        class  CBaseCtrl;
        class COMMU_API CCommApi
        {
        public:

            enum ECmd{
				Test=0,           //测试 DDS2ROS节点收到后应回复
                Takeoff=1,	      //起飞
                Land=2,	          // 降落
                Loiter=3,	      // 盘旋
                Return=4,         // 返航
                Point=5,          //指点飞行
                SetHome=6,        //对准坐标原点
                SetTargetLOc=7,   //设置目标位置信息
                DoTask=8,         //执行任务
                SetVideo=9,       //图片视频指令
                GetFtpInfo=10,    //获取文件传输信息
                GetID=11,         //
                SyncTime=12,      //时间同步
				SendHeartBeat = 254,//发送心跳
				UavType = 15,       // 1:fixwing 2:copter 3:vtol 4 car
				ImgCh = 19,         //切换图片通道 0：可见光，1：红外
				Weapon = 21,        //武器控制 1：爆炸 0:cancel
				Ping = 22           //ping


            };
			
			enum ERequest{
				Command=0,      //控制指令
				SetTaskStage,   //设置任务阶段
				AttackDisignObj //打击指定目标
			};

			struct SObjectReport{
				dm::uint8 id;        //飞机id
				dm::uint32 objid;    //目标身份id
				dm::uint16 cls;	     //目标分类
				dm::uint8  subcls;   //目标子分类
				dm::uint8  score;    //object score
				dm::int32 x;         //估计目标locx
				dm::int32 y;         //估计目标locy
				dm::int32 z;         //估计目标locz
				dm::int32 vx;        //估计目标速度vx
				dm::int32 vy;        //估计目标速度vy
				dm::int32 vz;        //估计目标速度vz
			};
			// struct 1
			struct SAttackObjectDesignate{
				dm::uint8  grpid;        //分组id
				std::vector<dm::uint8 > ids;  //分组内飞机id
				dm::uint8  type;        //打击方式
				SObjectReport objs; //需要打击目标信息
			};	
			// struct 2
			struct SCommandRequest
			{
			    dm::uint8  srcid;
			    dm::uint8  dstid;
			    dm::uint8  grpid;
			    dm::uint16 type;
			    dm::int32 param0;
			    dm::int32 param1;
			    dm::int32 param2;
			    dm::int32 param3;
			    dm::int32 param4;
			    float fparam5;
			    float fparam6;
			    float fparam7;
			    float fparam8;
			};
			// struct 3
			 struct SCommandResponse{
			    dm::uint8  id;                 //飞机id
				dm::uint8  src;                //指令发送源id
			    dm::uint16 cmd;       //指令类型
			    dm::uint16 subcmd;    //分指令
			    dm::uint16 status;    //指令执行状态
			    std::string rslt;              //附加信息

			};
			// struct 4
			struct SObjectFilter{
			    dm::uint8   type;       //0:关闭目标过滤功能,1:按目标内容(path指定图片文件路径)过滤,2:按目标分类过滤,3:不进行目标过滤
			    std::string path;      //图片文件路径，type为1时使用
			    dm::uint16  cls;       //目标分类
			    dm::uint8    subcls;    //目标子分类
			};
			// struct 5
			struct ObjectReport{
			    dm::uint8  id;
			    dm::uint32 objid;    //目标身份id
			    dm::uint16 cls;	     //目标分类
			    dm::uint8 subcls;   //目标子分类
                dm::uint8 score;    //目标得分
			    dm::int32 x;         //估计目标locx
			    dm::int32 y;         //估计目标locy
			    dm::int32 z;         //估计目标locz
			    dm::int32 vx;        //估计目标速度vx
			    dm::int32 vy;        //估计目标速度vy
			    dm::int32 vz;        //估计目标速度vz
			};

			// struct 6
			struct SOpticalPose{
			    float x;
			    float y;
			    float z;
				float roll;
			    float pitch;
			    float yaw;
			};

			// struct 7
			struct SParamShort{
				dm::uint8  type;       //参数、指令类型说明
				dm::int32 param1;     //参数1
				dm::int32 param2;     //参数2
				dm::int32 param3;     //参数3
				dm::int32 param4;     //参数4
			};

			// struct 8
			struct SSimpleVehicle
			{
				dm::uint64   time;             //时间 ms
				dm::uint8   typp;             //载具类型 
				dm::uint8    grp;             //分组编号
				dm::uint8    id;              //飞机id
				dm::uint8    bat;             //剩余电量
				dm::uint8    fix;             //gps fix状态
				dm::uint16  volt;             //电压 0.01v
				dm::uint8   sate;             //星数
				dm::uint8   lock;             //上锁状态
				dm::uint8   flymd;            //飞行模式
				dm::int32   roll;             //横滚 mrad
				dm::int32   pitch;            //俯仰 mrad
				dm::int32   yaw;              //偏航 mrad
				dm::int32   x;                //locx mm
				dm::int32   y;                //locy mm
				dm::int32   z;                //locz mm
				dm::int32   vx;               //速度 mm/s
				dm::int32   vy;      
				dm::int32   vz;      
				dm::int32   airspd;           //空速 mm/s
				dm::int32   lon;              //经度 deg E7
				dm::int32   lat;              //纬度 deg E7
				dm::int32   alt;              //海拔高度 mm
				dm::int32   radz;             //雷达高度 mm

			};
			// struct 9
			struct SStageHead{
				dm::uint8  grpid;             //分组编号
				std::vector<dm::uint8 > ids;     //同分组内全部id号
				dm::uint8  stage;             //任务阶段
				dm::uint8  tag;               //阶段标识
			};
			// struct 10
			struct SStatusTask{
			    dm::uint8  id;          //飞机id
			    dm::uint8  stage;	   //任务阶段
			    dm::uint8  tag;         //阶段标识 0:前突,1:搜索,2:跟踪打击,3:回收
			    dm::uint8  status;      //任务状态
			    dm::uint16 dstwaypt;   //目标航点id
			    dm::uint32 diswaypt;   //距离目标航点距离
			    dm::uint32 disobj;     //距离目标距离
			    dm::uint32 obj;        //目标id
			};

			// struct 11
			struct STaskStage{
				SStageHead head;//阶段任务头
				dm::uint8  scopetype;                  //区域范围类型
				std::vector<float> linx;             //航点x分量
				std::vector<float> liny;             //航点y分量
				std::vector<float> linz;             //航点z分量
				float maxspd;                     //最大飞行速度
				SParamShort trigger;               //任务阶段触发条件
				SParamShort form;                  //编队队形
				std::vector<float> formOffsetx;      //偏移队形时分组内每个飞机位置偏移x分量
				std::vector<float> formOffsety;      //偏移队形时分组内每个飞机位置偏移y分量
				std::vector<float> formOffsetz;      //偏移队形时分组内每个飞机位置偏移z分量
				SObjectFilter objs;                //目标信息
			};
			// struct 12
			struct SVehiWarning{
			    dm::uint8  id;     //飞机id
			    dm::uint8  src;    //告警信息来源 0:飞控,1:光电,2:雷达,3:武器
			    dm::uint16 tag;   //告警信息类别
			    dm::uint8   level; //告警级别
			    std::string desc;  //告警信息描述 
			};



            struct STargetInfo
            { 
                /* data */
                dm::uint64 utc;//utc毫秒
                int ch;        //图形摄像头通道号
                int sx;        //选框起始像素位置x
                int sy;        //选框起始像素位置y
                int len;       //选框长度
                int wid;       //选框宽度
            };	


			struct SAutoNvaline
            { 
                /* data */
                dm::uint8  id;     //飞机id
			    dm::uint8  type;    //告警信息来源 0:飞控,1:光电,2:雷达,3:武器
			    std::vector<int> x_lat;  //航线x(mm)或lat(deg*1e7)分量
                std::vector<int> y_lon;  //航线y(mm)或lon(deg*1e7)分量
                std::vector<int> z_alt;  //航线z(mm)或alt(mm)分量
            };	
			
			typedef std::function<void (const SSimpleVehicle &)> f_vehiCB_t;

			/*
			uint8 id;          //飞机id
			uint8 stage;	   //任务阶段
			uint8 tag;         //阶段标识 0:前突,1:搜索,2:跟踪打击,3:回收
			uint8 status;      //任务状态
			uint16 dstwaypt;   //目标航点id
			uint32 diswaypt;   //距离目标航点距离
			uint32 disobj;     //距离目标距离
			uint32 obj;        //目标id
			*/
			typedef std::function<void (dm::uint8,dm::uint8,dm::uint8,dm::uint8,dm::uint16,dm::uint32,dm::uint32,dm::uint32)> f_taskStatusCB_t; //任务状态回调函数类型
            
			/*
			uint8 id;     //飞机id
			uint8 src;    //告警信息来源 0:飞控,1:光电,2:雷达,3:武器
			uint16 tag;   //告警信息类别
			uint8  level; //告警级别
			string desc;  //告警信息描述
			*/
			typedef std::function<void (dm::uint8,dm::uint8,dm::uint16,dm::uint8,std::string)> f_warnCB_t; //告警信息回调函数类型

			/*
			dm::uint8  id;                 //回复飞机id
			dm::uint16 cmd;       //指令类型 ERequest
			dm::uint16 subcmd;    //分指令   ECmd
			dm::uint16 status;    //指令执行状态
			string rslt;              //附加信息
			*/
            typedef std::function<void (dm::uint8,dm::uint16,dm::uint16,dm::uint16,std::string)> f_respCB_t; //指令回复回调函数类型

			/*
			uint8 id;        //飞机id
			uint32 objid;    //目标身份id
			uint16 cls;	   //目标分类
			uint8  subcls;   //目标子分类
			uint8  score;   //目标得分
			int32 x;         //估计目标locx
			int32 y;         //估计目标locy
			int32 z;         //估计目标locz
			uint32 spd;      //估计目标移动速度			 
			*/
            typedef std::function<void (dm::uint8,dm::uint32,dm::uint16,dm::uint8,dm::uint8,dm::int32,dm::int32,dm::int32,dm::uint32)> f_objCB_t; //目标信息回调函数类型
			
			/*
			* id:上报飞机id
			* typ:航点类型0:loc,1:gps
			* 航线x(mm)或lat(deg*1e7)分量
			* 航线y(mm)或lon(deg*1e7)分量
			* 航线z(mm)或alt(mm)分量
			*/
			typedef std::function<void (dm::uint8,dm::uint8,std::vector<int>&,std::vector<int>&,std::vector<int>&)> f_autoLineCB_t;//自动规划航线回调函数类型

		private:
            /* data */
            static CBaseCtrl* m_commuCtrl;//实际通信对象指针
            static int m_refCount;        //实际通信对象引用计数
            SCommandRequest m_cmdInfo;    //存放控制指令信息           
			f_vehiCB_t m_vehiCB;          //简单飞控信息回调函数
			f_respCB_t m_respCB;		  //回复信息回调函数
			f_warnCB_t m_warnCB;          //告警回调函数			
			f_taskStatusCB_t m_taskStatusCB; //任务状态回调函数
			f_objCB_t m_objCB;               //识别目标回调函数
			f_autoLineCB_t m_lineCB;      //自动规划航线回调函数
			std::mutex m_mtx;
        public:
            CCommApi(f_vehiCB_t vehicleSipleCB,f_respCB_t cmdResCB,f_warnCB_t warnCB,f_taskStatusCB_t taskCB,f_objCB_t objCB);
            ~CCommApi();
			//test
			void commuTest(dm::uint8 dstId=255,dm::uint8 grpId=0,dm::uint8 srcId=0);

            /*发送控制指令信息
			*指令定义参考方案文档7.1.1
			*/
			void sendCommonInf(const SCommandRequest &inf);
			
			/*发送任务阶段信息 结构体
			* 参数信息参考方案文档8.1.2
			*/
			void sendTaskStage(const STaskStage &inf);
			
			/*发送指定目标跟踪、打击
			* 参数信息参考文档8.1.9
			*/
			void sendAttackObj(const SAttackObjectDesignate &inf);
			
			/*发送光学定位数据
			 *参数信息参考8.1.10
			*/
			void sendOptical(dm::uint8 id,const SOpticalPose &inf);

			/*
			*发送json格式任务阶段信息
			*/
			void sendJsonStage(const std::string &info);

			/*
			*设置自动上报航线回调函数
			*/
			void setAutoLineCB(f_autoLineCB_t autoLineCB);

			
			private:

			void onRcvObj(const CCommApi::SObjectReport &info);
			void onRcvWarn(const CCommApi::SVehiWarning &info);
			void onRcvTaskStatus(const CCommApi::SStatusTask& info);
        };
        
     
        
    } // namespace commu
    
    
} // namespace zyzn


#endif