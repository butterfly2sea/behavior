#include"DDSCtrl.h"
#include"Ros1Ctrl.h"


using namespace zyzn::commu;
CDDSCtrl *ddsCommu = nullptr;
CRos1Ctrl *ros1Commu = nullptr;

void onRcvResp(dm::uint8 id,dm::uint8 type,dm::int16 sts,std::string rslt){
    if(CCommApi::ECmd::Test == type && id >0){
        ros1Commu->addOptPoseSub(id);
    }
}

int main(int argc,char **argv){
    
    ddsCommu = new CDDSCtrl();
    
    ros1Commu = new CRos1Ctrl();
    dm::uint8 selfId = 0;
    
    if(ddsCommu && ros1Commu){
        ddsCommu->initOptNode();
        ros1Commu->optInfoInit(argc,argv,"OptInfoNode");
        //
        ros1Commu->sig_rcvOptPose.connect(boost::bind(&CDDSCtrl::sendOptPose,ddsCommu,boost::placeholders::_1,boost::placeholders::_2));

        
        ddsCommu->sig_rcvResp.connect(boost::bind(onRcvResp,boost::placeholders::_1,boost::placeholders::_2,boost::placeholders::_3,boost::placeholders::_4));
    }
    ros::Rate rate(500);
    bool sendTest = false;
    
    while(ros::ok()){
        if(!sendTest){
            sendTest = true;
            CCommApi::SCmdInfo info;
            memset(&info,0,sizeof(CCommApi::SCmdInfo));
            info.dstId = 0xFF;
            info.type = CCommApi::ECmd::Test;
            ddsCommu->sendCmdInfo(info);
            
        }
        ros::spinOnce();
        rate.sleep();
    }
    if(ddsCommu)
        delete ddsCommu;
    if(ros1Commu)
        delete ros1Commu;

    return 0;
}