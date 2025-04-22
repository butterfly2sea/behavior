
#include <log/Logger.hpp>
#include "../../include/set/SetDstPt.hpp"
#include "../../include/set/SetLine.hpp"
#include "../../include/get/GetLocation.hpp"
#include "../../include/info/OffboardInfo.hpp"
#include "../../include/info/Param.hpp"
#include "../../include/utility/Utility.hpp"

namespace zyzn{
    namespace set{

        //节点的inputs端口名，用于参数名获取
        static const char * g_inputNames[CSetDstPt::JsonParamIdx::PARAMS_COUNT]={
            "rdsParam","itvParam","altParam","ptTypParam","dstParam"
        };
        //默认json任务中参数名
        static const char * g_defParams[CSetDstPt::JsonParamIdx::PARAMS_COUNT]={
            "radius","intval","alt","pointTag","dstLoc"
        };
        
        CSetDstPt::SAutoDstPt CSetDstPt::m_s_autoDstPt;
        CSetDstPt::CSetDstPt(const std::string& name, const NodeConfig& config):SyncActionNode(name,config){
            updateParam();
        }

        PortsList CSetDstPt::providedPorts(){
            return { InputPort<int>("step"),//任务步骤
            InputPort<float>("obsHgh"),//避障高度
            InputPort<std::string>(g_inputNames[0]),//半径
            InputPort<std::string>(g_inputNames[1]),//间隔
            InputPort<std::string>(g_inputNames[2]),//高度
            InputPort<std::string>(g_inputNames[3]),//点类型
            InputPort<std::string>(g_inputNames[4]),//目标点
            OutputPort<custom_msgs::msg::OffboardCtrl>("target")};//期望位置
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
                    txtLog().info(THISMODULE "use radius via inputport is:%f",rds);
                }else if(set::CSetLine::arvDis() > 1){
                    rds = set::CSetLine::arvDis();
                    txtLog().info(THISMODULE "use radius via arvdis is:%f",rds);
                }else{
                    txtLog().info(THISMODULE "use radius via default is:%f",rds);
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
            txtLog().info(THISMODULE "step:%d x:%f y:%f z:%f",
                step,ctrl.x,ctrl.y,ctrl.z);             
            return NodeStatus::SUCCESS;
        }

        void CSetDstPt::updateParam(){
            //获取radius
            m_s_autoDstPt.rdis = plugin::BasePlugin::getValue(getParam(JsonParamIdx::RDS_IDX));
            //获取intval
            m_s_autoDstPt.intval = plugin::BasePlugin::getValue(getParam(JsonParamIdx::ITV_IDX));
            //获取alt
            m_s_autoDstPt.intval = plugin::BasePlugin::getValue(getParam(JsonParamIdx::ALT_IDX));
            //获取dstloc
            const Json::Value & dstLoc = getParam(JsonParamIdx::DST_IDX);
            if(dstLoc.isArray() && dstLoc.size()>0){
                const Json::Value & pt = dstLoc[0];
                if(pt.isMember("x_lat"))
                    m_s_autoDstPt.dstPt.x = plugin::BasePlugin::getValue(pt["x_lat"]);   
                if(pt.isMember("y_lon"))
                    m_s_autoDstPt.dstPt.y = plugin::BasePlugin::getValue(pt["y_lon"]);
                if(pt.isMember("z_alt"))
                    m_s_autoDstPt.dstPt.z = plugin::BasePlugin::getValue(pt["z_alt"]);
                algorithm::CUtility::checkZValid<float>(m_s_autoDstPt.dstPt.z);                  
            }  
            //获取点类型
            const Json::Value & ptTyp = getParam(JsonParamIdx::PT_TYP_IDX);
            int type = plugin::BasePlugin::Loc;
            if(!ptTyp.isNull() && ptTyp.isString()){
                type = ptTyp.asString()=="loc"?plugin::BasePlugin::Loc:plugin::BasePlugin::Gps;
            }
            if(plugin::BasePlugin::Gps == type){
                algorithm::CUtility::gps2loc(info::CParam::home(),m_s_autoDstPt.dstPt);
            }
        }

        const Json::Value & CSetDstPt::getParam(JsonParamIdx idx){
            if(idx < 0 || idx > JsonParamIdx::PARAMS_COUNT)
                idx = (JsonParamIdx)0;
            std::string paramName = g_defParams[idx];
            getInput<std::string>(g_inputNames[idx],paramName);
            return info::CParam::getParam(paramName);
        }
    }
}