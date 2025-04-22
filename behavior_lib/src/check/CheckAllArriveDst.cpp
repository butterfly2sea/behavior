
#include <check/CheckAllArriveDst.hpp>
#include <set/SetLine.hpp>
#include <info/Param.hpp>
#include <log/Logger.hpp>
namespace zyzn {
    namespace check{
        PortsList CCheckAllArriveDst::providedPorts(){
            return { 
                    InputPort<int>("delay"),
                    OutputPort<float>("dstyaw")
            };
                        
        }

        CCheckAllArriveDst::CCheckAllArriveDst(const std::string& name, const NodeConfig& config):SyncActionNode(name,config),
        m_delay(std::chrono::nanoseconds(0)){
            //初始化分组内全部飞机航向信息
            initInfo();
            //外部飞机信息订阅使用行为控制程序的rosnode，以持续更新外部位置
            std::string tpName = "outer/information/simple_vehicle";
            m_subOutVehi = zyzn::info::CParam::rosNode()->create_subscription<custom_msgs::msg::SimpleVehicle>(
            tpName,rclcpp::SensorDataQoS(),
            std::bind(&CCheckAllArriveDst::outSimpleVehiCB,this,std::placeholders::_1));
        }

        NodeStatus CCheckAllArriveDst::tick(){
            
            if(set::CSetLine::FixWing == get::CGetlocation::simpVehi().type || 
            set::CSetLine::VtolFix == get::CGetlocation::simpVehi().type || 
            set::CSetLine::Coper == set::CSetLine::vehiType()){
                txtLog().info(THISMODULE "no need to fixwing");
                return NodeStatus::SUCCESS;
            }
            
            //设置本机目的航向
            if(!isSetDstYaw(info::CParam::vehiId())){                       
                float yaw = getDstYaw(info::CParam::vehiId(),get::CGetlocation::simpVehi().x/1e3,get::CGetlocation::simpVehi().y/1e3);
                setDstYaw(info::CParam::vehiId(),yaw);
                txtLog().info(THISMODULE "set self dst yaw:%f",yaw);
            } 
            m_info[info::CParam::vehiId()].curYaw = get::CGetlocation::simpVehi().yaw/1e3;
            //是否已经获取了同分组内飞机信息
            if(m_info.size() < set::CSetLine::ids().size()){
                RCLCPP_WARN(rclcpp::get_logger("CheckAllArvDst"),"not get all yaw info of same grp");
                return NodeStatus::FAILURE;
            }
            float yawDiff = 0;
            if(isSelfInYaw(yawDiff)){
                if(isAllOutInYaw()){
                    if(m_delay.count() == 0){//第一次判定全部就绪后 开始延时 
                        int t = 5000;
                        getInput<int>("delay",t);         
                        int64_t delay = t;//ms
                        std::chrono::nanoseconds dl(delay * 1000000);
                        m_delay = dl;  
                        m_tp  = std::chrono::system_clock::now();
                        //yawDiff = algorithm::CUtility::getRadDiff(m_selfInfo.yaw,m_info[info::CParam::m_sSelfId].dstYaw);
                        txtLog().info(THISMODULE 
                        "arrive dst yaw with other,init delay:%ld selfyaw:%f diff:%f",
                        m_delay.count(),m_info[info::CParam::vehiId()].curYaw,yawDiff);
                    }
                }
                //延迟到达后认为到达目的航向
                if(m_delay.count() != 0){
                    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();                       
                    if((tp-m_tp) > m_delay){
                        txtLog().info(THISMODULE "arrive dst yaw with other");
                        m_delay = std::chrono::nanoseconds(0);
                        //vehiType = info::CParam::m_sVehiType;
                        return NodeStatus::SUCCESS;
                    }
                }
            }
            updateCtrlYaw(yawDiff);                   
            return NodeStatus::FAILURE;
        }

        bool CCheckAllArriveDst::isSelfInYaw(float &yawDiff){
            yawDiff = algorithm::CUtility::getRadVel(m_info[info::CParam::vehiId()].curYaw,m_info[info::CParam::vehiId()].dstYaw);
            return isInYaw(yawDiff);
        }

        bool CCheckAllArriveDst::isAllOutInYaw(){
            for(auto it=m_info.begin();it!=m_info.end();++it){
                float yawDiff = algorithm::CUtility::getRadVel(it->second.curYaw,it->second.dstYaw);
                if(!isInYaw(yawDiff))//有一架飞机未到达则返回false
                    return false;
            }
            return true;
        }

        bool CCheckAllArriveDst::isSetDstYaw(const uint8_t & id){
            if(m_info.find(id) == m_info.end())
                return false;
            if(m_info[id].dstYaw >= (InitYaw-1))
                return false;
            return true;
        }

        float CCheckAllArriveDst::getDstYaw(const uint8_t & id,float curx,float cury){
            //如果航线具有航点将第一个航点作为目的位置
            if(set::CSetLine::wayPts().points.size()>0){
                float dstx = set::CSetLine::wayPts().points[0].x;
                float dsty = set::CSetLine::wayPts().points[0].y;
                for(int i=0;i<set::CSetLine::ids().size();++i){
                    if(set::CSetLine::ids()[i] == id && 
                    set::CSetLine::offsets().points.size()>i){
                        dstx += set::CSetLine::offsets().points[i].x;
                        dsty += set::CSetLine::offsets().points[i].y;
                        break;
                    }
                }
                return algorithm::CUtility::getYawN2PFrmLoc(curx,cury,dstx,dsty);

            }
            if(m_info.find(id) != m_info.end()){//如果没有航线，则将当前航向当目的航向
                return m_info[id].curYaw;
            }
            return 0;//信息中没此飞机信息则返回0
        }

        void CCheckAllArriveDst::updateCtrlYaw(const float & yawDiff){
            
            //if(isInYaw(yawDiff)){
                setOutput<float>("dstyaw",m_info[info::CParam::vehiId()].dstYaw);
                
            //}
            /*else{
                setOutput<float>("dstyaw",m_info[info::CParam::m_sSelfId].curYaw+yawDiff/3.0);
                    if(ii>100){
                    txtLog().info(THISMODULE "out yaw diff:%f dst:%f cur:%f",yawDiff,m_info[info::CParam::m_sSelfId].curYaw+yawDiff/3.0,
                    m_info[info::CParam::m_sSelfId].curYaw);
                    ii = 0;
                }
                                    
            }*/
            
            //setOutput<custom_msgs::msg::OffboardCtrl>("target",m_target);
        }

        void CCheckAllArriveDst::outSimpleVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg){

            if(msg){ 
                //只处理同分组内飞机信息  
                if(m_info.find(msg->id) == m_info.end()){
                    return;
                }       
                if(!isSetDstYaw(msg->id)){//如未设置目的航向，则先依据航点、偏移及当前位置计算目的偏航
                    float yaw = getDstYaw(msg->id,msg->x/1e3,msg->y/1e3);
                    setDstYaw(msg->id,yaw);
                } 
                m_info[msg->id].curYaw = msg->yaw/1e3;                                        
            }
        }

        void CCheckAllArriveDst::initInfo(){
            for(int i=0;i<set::CSetLine::ids().size();++i){
                m_info[set::CSetLine::ids()[i]].dstYaw = InitYaw;
            }
        }


    }
}