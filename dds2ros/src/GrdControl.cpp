#include <chrono>
#include"CommApi.h"
#include<iostream>
#include<json/json.h>
#include<iostream>
#include<fstream>
#include<map>
#include<vector>
#include<fstream>
//#include<HttpSendVehiInfo.h>
#include <time.h>
//#include <windows.h>
using namespace zyzn::commu;
CCommApi * g_commu = nullptr;
dm::uint8 g_id = 0;
double g_lat,g_lon;
float g_airSpd,g_yaw,g_alt;
bool prtSimp = false;
bool prtObj = false;
int selectObjId = -1;
std::map<int,CCommApi::SAttackObjectDesignate> attackObjs;
//zyzn::http::CHttpVehiInfoApi *psndVehi = nullptr;
class CTest1{
    public:
    void vehicleSimpleCB(const CCommApi::SSimpleVehicle &info){
		if (prtSimp) {
			static int cnt = 0;
			++cnt;
			std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
			std::cout << "simple vehicleCB id:" << info.id << "cnt:"<<cnt<<""<<t.time_since_epoch().count()<<" x:" << info.x << " y:" << info.y << " z:" << info.z << "\n";
		}
		//SYSTEMTIME wtm;
		//GetLocalTime(&wtm);
		
		//std::cout << "time:" << wtm.wSecond << "." << wtm.wMilliseconds << "cnt:" << cnt << "\n";
		//char sinfo[64] = { 0 };
		double lat = info.lat / 1e7;
		double lon = info.lon / 1e7;
		//sprintf(sinfo, "lon:%.7f lat:%.7f", lon, lat);
		//std::cout << sinfo << std::endl;
		//if (psndVehi) {
		//	psndVehi->sendObj( lon, lat, info.id);
		//}

}
	void cmdRespCB(dm::uint8 id, dm::uint16 type, dm::uint16 subType, dm::uint16 sts, std::string info) {
		std::cout << "cmdRespCB id:" << id << " type:" << type << " subType:" << subType << " sts:" << sts << " info:" << info.c_str() << std::endl;

	}

	void lineRptCB(dm::uint8 id, dm::uint8 typ, std::vector<int>& x_lat, std::vector<int>& y_lon, std::vector<int>& z_alt) {
		std::cout << "line reportCB id:" << id << " type:" << typ << " x_lat.size:" << x_lat.size() << "y_lon.size:" << y_lon.size() << "z_alt.size:" << z_alt.size() << std::endl;
	}

	void rcvObjCB(dm::uint8 id, dm::uint32 objId, dm::uint16 cls, dm::uint8 subCls, dm::int32 x, dm::int32 y, dm::int32 z, dm::uint32 spd) {
		attackObjs[objId].objs.id = id;
		attackObjs[objId].objs.objid = objId;
		attackObjs[objId].objs.cls = cls;
		attackObjs[objId].objs.subcls = subCls;
		attackObjs[objId].objs.x = x;
		attackObjs[objId].objs.y = y;
		attackObjs[objId].objs.z = z;
		attackObjs[objId].grpid = 0;
		attackObjs[objId].ids.clear();
		attackObjs[objId].ids.push_back(1);
		
		if (prtObj) {
			std::cout << "rcvObjCB id: " << ((int)id) << " objId:" << objId << " cls:" << cls << " subCls:" << subCls << " x:" << x << " y:" << y << " z:" << z << " spd:" << spd << std::endl;
		}
	}
	void rcvWarnCB(dm::uint8 id, dm::uint8 src, dm::uint16 tag, dm::uint8 level, std::string info) {
		std::cout << "rcvWarnCB id:" << id << " src:" << src << " tag:" << tag << " level:" << level << " info:" << info.c_str() << std::endl;
	}
	void rcvTaskStatusCB(dm::uint8, dm::uint8, dm::uint8, dm::uint8, dm::uint16, dm::uint32, dm::uint32, dm::uint32) {

	}

};

class CTest2{
    public:
    void cmdRespCB(dm::uint8 id,dm::uint16 type,dm::uint16 subType,dm::uint16 sts,std::string info){
    std::cout<<"cmdRespCB id: "<<id<<" status: "<<sts<<" type: "<<type<<" info:"<<info<<std::endl;
    
}
void cmdRespCB2(dm::uint8 id,dm::uint16 type,dm::uint16 subType,dm::uint16 sts,std::string info){
    std::cout<<"cmdRespCB2 id: "<<id<<" status: "<<sts<<" type: "<<type<<" info:"<<info<<std::endl;
    
}
};



void getLonLatAlt(){
    std::cout<<"请输入经度\n";
    std::cin>>g_lon;
    std::cout<<"请输入纬度\n";
    std::cin>>g_lat;
    std::cout<<"请输入海拔高度，米\n";
    std::cin>>g_alt;
}

void getSpdYawAlt(){
    std::cout<<"请输入空速，米每秒\n";
    std::cin>>g_airSpd;
    std::cout<<"请输入航向角，弧度\n";
    std::cin>>g_yaw;
    std::cout<<"请输入海拔高度，米\n";
    std::cin>>g_alt;
}

int main(int argc,char **argv){
	system("chcp 65001");
	//zyzn::http::CHttpVehiInfoApi sndVehi("48.16.16.210", 8089, "/api/Hkobject/postWRJ");
	//zyzn::http::CHttpVehiInfoApi sndVehi("127.0.0.1", 7006, "/api/Hkobject/postWRJ");
	//psndVehi = &sndVehi;


    CTest1 t1;
    CTest2 t2;
	
	Json::Value jsTest;
	Json::String errs;
	std::ifstream istr;
	std::cout << "argv[0]:" << argv[0] << std::endl;
	std::string exePath = argv[0];
	int pos = exePath.rfind('\\');
	
	std::cout << "pos:" << pos << std::endl;
	std::string fileName("test.json");
	std::string jsonExample;
	if (pos > 0)
		fileName = exePath.substr(0, pos + 1) + "test.json";
	istr.open(fileName.c_str(), std::ifstream::in);
	if (istr.is_open()) {
		Json::CharReaderBuilder builder;
		if (!Json::parseFromStream(builder, istr, &jsTest, &errs)) {
			std::cout << "read json frm file:" << fileName.c_str() << "failed"<<std::endl;

		}
		Json::StreamWriterBuilder bd;
		jsonExample = Json::FastWriter().write(jsTest);//Json::writeString(bd, jsTest);//jsTest.toStyledString();
		
		
		std::cout << "jsTest:" << jsonExample << std::endl << " len:" << jsonExample.length();
	}
	else {
		std::cout << "open file:" << fileName.c_str() << " failed"<<std::endl;
	}
    
    CCommApi::SCommandRequest sendInf;
    CCommApi commuApi(std::bind(&CTest1::vehicleSimpleCB,t1,std::placeholders::_1),std::bind(&CTest1::cmdRespCB,t1,std::placeholders::_1,
    std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5),
		std::bind(&CTest1::rcvWarnCB,t1, std::placeholders::_1,
			std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
		std::bind(&CTest1::rcvTaskStatusCB,t1, std::placeholders::_1,
			std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8),
		std::bind(&CTest1::rcvObjCB,t1, std::placeholders::_1,
			std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8));

    //CCommApi commuApi2(std::bind(&CTest1::vehicleSimpleCB,t1,std::placeholders::_1),std::bind(&CTest2::cmdRespCB2,t2,std::placeholders::_1,
    //std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
	commuApi.setAutoLineCB(std::bind(&CTest1::lineRptCB,t1,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
	CCommApi::STaskStage preStage;
	preStage.head.grpid = 1;
	preStage.head.ids.push_back(1);
	preStage.head.stage = 1;
	preStage.head.tag = 0;//
	preStage.trigger.type = 3;

	preStage.form.type = 0;
	preStage.formOffsetx.push_back(0);
	preStage.formOffsety.push_back(0);
	preStage.formOffsetz.push_back(0);
	preStage.linx.resize(3);
	preStage.liny.resize(3);
	preStage.linz.resize(3);
	preStage.linx[0] = 0;
	preStage.liny[0] = 0;
    preStage.liny[1] = 0;
	preStage.linx[1] = preStage.linx[2]  = -3;//50;
	preStage.liny[2] = -1;
	preStage.linz[0] = preStage.linz[1] = preStage.linz[2] = -1;
	preStage.maxspd = 0.5;


	//
	CCommApi::STaskStage preStage2;
	preStage2.head.grpid = 1;
	preStage2.head.ids.push_back(1);
	preStage2.head.ids.push_back(2);
	preStage2.head.stage = 1;
	preStage2.head.tag = 0;//
	preStage2.trigger.type = 3;

	preStage2.form.type = 6;
	preStage2.formOffsetx.push_back(0);
	preStage2.formOffsety.push_back(-0.5);
	preStage2.formOffsetz.push_back(0);


	preStage2.formOffsetx.push_back(0);
	preStage2.formOffsety.push_back(0.5);
	preStage2.formOffsetz.push_back(0);

	preStage2.linx.resize(3);
	preStage2.liny.resize(3);
	preStage2.linz.resize(3);
	preStage2.linx[0] = 0;
	preStage2.liny[0] = 0;
	preStage2.liny[1] = 0;
	preStage2.linx[1] = preStage2.linx[2] = -3;//50;
	preStage2.liny[2] = -1;
	preStage2.linz[0] = preStage2.linz[1] = preStage2.linz[2] = -1;
	preStage2.maxspd = 0.5;
	
	CCommApi::STaskStage schStage;
	schStage.head.grpid = 2;
	schStage.head.ids.push_back(1);
	schStage.head.stage = 2;
	schStage.head.tag = 1;//
	schStage.form.type = 0;
	schStage.trigger.type = 3;
	schStage.formOffsetx.push_back(0);
	schStage.formOffsety.push_back(0);
	schStage.formOffsetz.push_back(0);
	schStage.linx.resize(3);
	schStage.liny.resize(3);
	schStage.linz.resize(3);
	schStage.linx[0] = 50;
	schStage.liny[0] = schStage.liny[1] = 100;
	schStage.linx[1] = schStage.linx[2] = 0;
	schStage.liny[2] = 200;

	CCommApi::STaskStage traStage;
	traStage.head.grpid = 3;
	traStage.head.ids.push_back(1);
	traStage.head.stage = 3;
	traStage.head.tag = 2;
	traStage.form.type = 0;
	traStage.trigger.type = 3;
	traStage.formOffsetx.push_back(0);
	traStage.formOffsety.push_back(0);
	traStage.formOffsetz.push_back(0);


	CCommApi::STaskStage attackStage;
	attackStage.head.grpid = 3;
	attackStage.head.ids.push_back(1);
	attackStage.head.stage = 4;
	attackStage.head.tag = 3;
	attackStage.form.type = 0;
	attackStage.trigger.type = 3;
	attackStage.formOffsetx.push_back(0);
	attackStage.formOffsety.push_back(0);
	attackStage.formOffsetz.push_back(0);

	CCommApi::SOpticalPose optPose;
	optPose.x = 1;
	optPose.y = 2;
	optPose.z = 0;





    //_commu = &commuApi;
	std::string stest("{}");//(jsTest.asCString());
	//std::cout << "stest:" << stest;
    int ic=999;
	int id = 0;
	//while (true)
	//{
		//sndVehi.sendObj( 40.132987,116.155187, 6);
		//_sleep(1);
	//}
    while(ic != 99){
        std::cout<<"**********请输入数字以进行对应操作 **********\n";
		std::cout<<"**********退出程序:99              **********\n";
        std::cout<<"**********通信测试:0              **********\n";
        std::cout<<"**********起飞:1                   **********\n";
        std::cout<<"**********降落:2                   **********\n";
        std::cout<<"**********盘旋:3                   **********\n";
        std::cout<<"**********返航:4                   **********\n";
        std::cout<<"**********对准坐标:5               **********\n";
        std::cout<<"**********飞航线:6                 **********\n";
        std::cout<<"**********开始碰撞:7               **********\n";
        std::cout<<"**********停止碰撞:8               **********\n";
        std::cout<<"**********单机1设置起飞前出任务阶段:90  **********\n";
		
		//std::cout <<"*********设置起飞搜索任务阶段:91  **********\n";
		//std::cout <<"*********设置起飞前跟踪务阶段:92  **********\n";
		//std::cout <<"*********设置起飞返航任务阶段:93  **********\n";
		std::cout <<"*********单机1开始起飞前出任务阶段:100  **********\n";
		std::cout << std::endl << std::endl;
		std::cout <<"**********多机1 2设置起飞前出任务阶段:96  **********\n";
		std::cout <<"*********多机1  2开始起飞前出任务阶段:106 **********\n";

		std::cout << std::endl << std::endl;

		std::cout <<"*********暂停起飞前出任务阶段:101  **********\n";
		std::cout <<"*********继续起飞前出任务阶段:102  **********\n";
		std::cout <<"*********停止起飞前出任务阶段:103  **********\n";

		std::cout << std::endl << std::endl;
		std::cout << "*********设置打击当前目标:120  **********\n";
		std::cout << "*********开始打击目标:121  **********\n";

		std::cout<<  "*********设置json任务:151  **********\n";

		std::cout <<"*********测试光学定位数据:200  **********\n";
		std::cout <<"*********打印简单飞控消息回调:201  **********\n";
		std::cout <<"*********关闭打印简单飞控消息回调:202  **********\n";
		std::cout << "*********打印目标消息回调:203  **********\n";
		std::cout << "*********关闭打印目标消息回调:204  **********\n";

		std::cout << std::endl;
		std::cout << "*********获取rtsp地址:210  **********\n";

		std::cout << std::endl;
		std::cout << "*********停止视频:211  **********\n";

		
        std::cin>>ic;
		bool sendcomm = true;
        if(99==ic){
            std::cout<<"**********程序退出                 **********\n";
            return 0;
        }
        if(ic<0 ){
            std::cout<<"暂不支持的控制\n";
            continue;
        }
        
            
        std::cout<<"请输入目标id\n";
        std::cin>>id;
		g_id = id;
        sendInf.dstid=g_id;
        bool needSend = true;
		std::cout << "ic:" << ic << "\n";
        switch (ic)
        {
        case 0:
            sendInf.type = CCommApi::ECmd::Test;
            break;
        case 1:
            getLonLatAlt();
            sendInf.type = CCommApi::ECmd::Takeoff;
            sendInf.param0 = g_lat;
            sendInf.param1 = g_lon;
            sendInf.param2 = g_alt;
            break;
        case 2:
            sendInf.type = CCommApi::ECmd::Land;
            break;
        case 3:
            sendInf.type = CCommApi::ECmd::Loiter;
            break;
        case 4:
            sendInf.type = CCommApi::ECmd::Return;
            break;
        case 5:
            sendInf.type = CCommApi::ECmd::SetHome;
            break;
        case 6:
            //sendInf.type = CCommApi::ECmd::FlyTask;
            break;
        case 7:
            //sendInf.type = CCommApi::ECmd::st;
            break;
        case 8:
            //commuApi.stopColliding(g_id);
            break;
        case 9:
            //commuApi.setNavlineByFile("text.xml",g_id);
		case 90://设置起飞前出
			std::cout << "单机1前出阶段设置\n";
			commuApi.sendTaskStage(preStage);
			sendcomm = false;
            break;
		case 96://设置多机起飞前出
			std::cout << "多机1   2前出阶段设置\n";
			commuApi.sendTaskStage(preStage2);
			sendcomm = false;
			break;

		case 100://开始前出阶段飞行
			std::cout << "单机飞行前出开始\n";
			sendInf.type = 8;
			sendInf.param0 = 0;//开始
			sendInf.param1 = 1;
			sendInf.param2 = 0;
			break;
		case 106://多机开始起飞前出
			
			sendInf.type = 8;
			sendInf.param0 = 0;//开始
			sendInf.param1 = 1;
			sendInf.param2 = 0;
			
			break;
		case 101://暂停
			sendInf.type = 8;
			sendInf.param0 = 1;//暂停
			sendInf.param1 = 1;
			sendInf.param2 = 0;
			break;
		case 102://继续
			sendInf.type = 8;
			sendInf.param0 = 2;//继续
			sendInf.param1 = 1;
			sendInf.param2 = 0;
			break;
		case 103://停止
			sendInf.type = 8;
			sendInf.param0 = 3;//停止
			sendInf.param1 = 1;
			sendInf.param2 = 0;
			break;
		case 120://set attack obj
		{
			std::cout << "available objs:" << std::endl;

			for (auto it = attackObjs.begin(); it != attackObjs.end(); it++) {
				std::cout << "objId:" << it->second.objs.objid << " x:" << it->second.objs.x << " y:" << it->second.objs.y << " z:" << it->second.objs.z << std::endl;
			}
			selectObjId = -1;
			std::cin >> selectObjId;
			if (attackObjs.find(selectObjId) != attackObjs.end()) {
				commuApi.sendAttackObj(attackObjs[selectObjId]);
				//commuApi.sendTaskStage(attackStage);

			}
			else {
				std::cout << "wrong objid" << std::endl;

			}

			sendInf.type = 8;
			sendInf.param0 = 0;//开始
			sendInf.param1 = 4;
			sendInf.param2 = 3;
			sendcomm = false;
			//sendcomm = false;
			break;
		}
		case 121: //start attack
		{
			if (attackObjs.size() > 0 && -1 == selectObjId) {
				selectObjId = attackObjs.begin()->first;
			}
			if (attackObjs.find(selectObjId) != attackObjs.end()) {
				commuApi.sendAttackObj(attackObjs[selectObjId]);
				commuApi.sendTaskStage(attackStage);

				//_sleep(10);
				sendInf.type = 8;
				sendInf.param0 = 0;
				sendInf.param1 = 4;
				sendInf.param2 = 3;
			}
			else {
				sendcomm = false;
			}

			break;
		}
		case 151:
			
			commuApi.sendJsonStage(jsonExample);
			sendcomm = false;
			break;
		case 200://optpose 
			commuApi.sendOptical(1, optPose);
			sendcomm = false;
			break;
		case 201:
			prtSimp = true;
			sendcomm = false;
			break;
		case 202:
			prtSimp = false;
			sendcomm = false;
			break;
		case 203:
			prtObj = true;
			sendcomm = false;
			break;
		case 204:
			prtObj = false;
			sendcomm = false;
			break;
		case 210:
			sendInf.type = 9;//CCommApi::ECmd::SetVideo;
			sendInf.param0 = 2;
			break;
		case 211:
			sendInf.type = 9;//CCommApi::ECmd::SetVideo;
			sendInf.param0 = 3;
			break;

        default:
            needSend = false;
            break;
        }
		std::cout << "sendcomm:" << sendcomm << std::endl;
		if (sendcomm) {
			std::cout << "type:" << (int)(sendInf.type)<<"\n";
			commuApi.sendCommonInf(sendInf);
		}
        //commuApi2.sendCommonInf(sendInf);
        
    }
    return 0;
}


