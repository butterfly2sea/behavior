#include"DDSCtrl.h"
#include"Ros2Ctrl.h"


using namespace zyzn::commu;

int main(int argc,char **argv){
    setlocale(LC_ALL, "");
    
    CDDSCtrl *ddsCommu = new CDDSCtrl();

    //CRos2Ctrl *ros1Commu = new CRos2Ctrl("DDS2RosNode");
    dm::uint8 selfId = 0;
    rclcpp::init(argc,argv);
    
    std::shared_ptr<CRos2Ctrl> rosCommu = std::make_shared<CRos2Ctrl>("DDS2ROSNode");

    
    if(ddsCommu && rosCommu){ 
    
        ddsCommu->initByDefault();
        rosCommu->init();
     //
        //rosCommu->sig_rcvAuxi.connect(boost::bind(&CDDSCtrl::sendAuxiInfo,ddsCommu,boost::placeholders::_1));
        rosCommu->sig_rcvVehicleSimple.connect(boost::bind(&CDDSCtrl::sendSimpleVehicle,ddsCommu,boost::placeholders::_1));
        rosCommu->sig_rcvResp.connect(boost::bind(&CDDSCtrl::sendResponse,ddsCommu,boost::placeholders::_1,boost::placeholders::_2,boost::placeholders::_3,boost::placeholders::_4,boost::placeholders::_5));
        rosCommu->sig_rcvWarning.connect(boost::bind(&CDDSCtrl::sendVehiWarning,ddsCommu,boost::placeholders::_1));
        rosCommu->sig_rcvObjComputation.connect(boost::bind(&CDDSCtrl::sendObjectComputation,ddsCommu,boost::placeholders::_1));
        rosCommu->sig_rcvStatusTask.connect(boost::bind(&CDDSCtrl::sendStatusTask,ddsCommu,boost::placeholders::_1));
        rosCommu->sig_rcvAutoNva.connect(boost::bind(&CDDSCtrl::sendAutoLine,ddsCommu,boost::placeholders::_1));

        //
        ddsCommu->sig_rcvAtact.connect(boost::bind(&CRos2Ctrl::sendAttactInfo,rosCommu,boost::placeholders::_1));
        //ddsCommu->sig_rcvAuxi.connect(boost::bind(&CRos2Ctrl::sendAuxiInfo,rosCommu,boost::placeholders::_1));
        ddsCommu->sig_rcvVehicleSimple.connect(boost::bind(&CRos2Ctrl::sendVehicleSimple,rosCommu,boost::placeholders::_1));
        ddsCommu->sig_rcvCmd.connect(boost::bind(&CRos2Ctrl::sendCmdInfo,rosCommu,boost::placeholders::_1));
        //ddsCommu->sig_rcvNavlineArea.connect(boost::bind(&CRos2Ctrl::sendNavlineArea,ros2Commu,boost::placeholders::_1,boost::placeholders::_2,boost::placeholders::_3,boost::placeholders::_4));
        ddsCommu->sig_rcvOptPose.connect(boost::bind(&CRos2Ctrl::sendOptPose,rosCommu,boost::placeholders::_1,boost::placeholders::_2));
        ddsCommu->sig_rcvTaskStage.connect(boost::bind(&CRos2Ctrl::sendTaskStage,rosCommu,boost::placeholders::_1));
        //ddsCommu->sig_rcvObjComputation.connect(boost::bind(&CRos2Ctrl::sendObjectComputation,rosCommu,boost::placeholders::_1));
        ddsCommu->sig_rcvStageSet.connect(boost::bind(&CRos2Ctrl::sendStageSet,rosCommu,boost::placeholders::_1));
        
    }
   
    rclcpp::spin(rosCommu);
    rclcpp::shutdown();
    if(ddsCommu)
        delete ddsCommu;


    return 0;
}