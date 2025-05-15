#include "../../include/set/SetDstPt.hpp"
#include "../../include/get/GetLocation.hpp"
#include "../../include/utility/Utility.hpp"
#include "../../include/set/SetLine.hpp"
#include "../../include/info/OffboardInfo.hpp"
namespace zyzn{
    namespace set{
        
        CSetDstPt::SAutoDstPt CSetDstPt::m_s_autoDstPt;
        CSetDstPt::CSetDstPt(const std::string& name, const NodeConfig& config):SyncActionNode(name,config){

        }


        PortsList CSetDstPt::providedPorts(){
            return { InputPort<int>("step"),
            InputPort<float>("obsHgh"),
            OutputPort<custom_msgs::msg::OffboardCtrl>("target")};
        }

        NodeStatus CSetDstPt::tick(){
            custom_msgs::msg::OffboardCtrl ctrl;
            int step = CurHorObsHgh;
            float obsHgh = ObsHgh;
            getInput<int>("step",step);
            getInput<float>("obsHgh",obsHgh);
            
            //依次从当前水平位置调整高度到障碍高度->到目的水平障碍高度位置->目的位置
            ctrl.ordmask = EOffboardMask::LocCtrl+EOffboardMask::YawCtrl;
            //默认为步骤为目的位置(期望位置)
            ctrl.x = m_s_autoDstPt.dstPt.x;
            ctrl.y = m_s_autoDstPt.dstPt.y;
            ctrl.z = m_s_autoDstPt.dstPt.z;
            if(CurHorObsHgh == step){//步骤为当前水平位置障碍高度(期望位置)
                ctrl.x = get::CGetlocation::simpVehi().x/1e3;
                ctrl.y = get::CGetlocation::simpVehi().y/1e3;
                ctrl.z = obsHgh;
            }else if(DstHorObsHgh == step){//步骤为目的水平障碍高度(期望位置)
                ctrl.z = obsHgh;
            }else if(DstHorCurHgh == step){//步骤为目的水平位置当前高度(期望位置)
                ctrl.z = get::CGetlocation::simpVehi().z/1e3;
            }
            algorithm::CUtility::checkZValid<float>(ctrl.z);
            if(set::CSetLine::FixWing == set::CSetLine::vehiType()){
                //当目标机型为固定翼时，mask为loc+空速，且空速值为0飞控会使用最小空速
                ctrl.ordmask = EOffboardMask::FixLocRadCtrl;//EOffboardMask::LocCtrl+EOffboardMask::AirSpdCtrl;
                ctrl.airspd = 0;
                double rds = 60;
                if(getInput<double>("radius",rds)){
                    RCLCPP_INFO(rclcpp::get_logger("SetDstPt"),"use radius via inputport is:%f",rds);
                }else if(set::CSetLine::arvDis() > 1){
                    rds = set::CSetLine::arvDis();
                    RCLCPP_INFO(rclcpp::get_logger("SetDstPt"),"use radius via arvdis is:%f",rds);
                }else{
                        RCLCPP_INFO(rclcpp::get_logger("SetDstPt"),"use radius via default is:%f",rds);
                }
                ctrl.vy = rds;

            }else{
                //当为旋翼时则需要计算当前位置和目的位置的航向,如相距很近则使用当前航向
                if(algorithm::CUtility::getDisFrmLoc(get::CGetlocation::simpVehi().x/1e3,
                get::CGetlocation::simpVehi().y/1e3,ctrl.x,ctrl.y)<0.5){
                    ctrl.yaw = get::CGetlocation::simpVehi().yaw/1e3;
                }else{
                    ctrl.yaw = algorithm::CUtility::getYawN2PFrmLoc(get::CGetlocation::simpVehi().x/1e3,
                    get::CGetlocation::simpVehi().y/1e3,ctrl.x,ctrl.y);
                }
                           
            }
            setOutput<custom_msgs::msg::OffboardCtrl>("target",ctrl);//设置输出参数  
            RCLCPP_INFO(rclcpp::get_logger("SetDstPt"),"step:%d x:%f y:%f z:%f",
                step,ctrl.x,ctrl.y,ctrl.z);             
            return NodeStatus::SUCCESS;
        }
    }
}