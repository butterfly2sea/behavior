#include <status/TraceStatus.hpp>
#include <set/SetTraceAttackObj.hpp>
#include <behavior_lib/info/Param.hpp>
#include <control/CameraCtrl.hpp>

#include <behavior_lib/utility/Utility.hpp>
#include <behavior_lib/get/GetGroupLocation.hpp>
#include <behavior_lib/info/OffboardInfo.hpp>
namespace zyzn{
    namespace status{
        CTraceStatus::SMixInfo CTraceStatus::m_s_mixObj;
        PortsList CTraceStatus::providedPorts(){
            return {OutputPort<int>("state"),//跟踪状态
            OutputPort<int>("ctrltyp"),//吊舱控制类型
            OutputPort<int>("param1"),//吊舱控制参数1、吊舱gps指向控制lat
            OutputPort<int>("param2"),//吊舱控制参数2、吊舱gps指向控制lon
            OutputPort<int>("param3"),//吊舱gps指向控制alt
            OutputPort<custom_msgs::msg::OffboardCtrl>("target"),//输出offboard控制量
            InputPort<float>("angle"),//相差角度
            InputPort<float>("tracez"),//跟踪高度
            InputPort<int>("followid"),//跟随飞机id和此飞机保持 相差角度（可能不用）
            InputPort<int>("mixloss"),//判定融合目标丢失时长毫秒,默认4000
            InputPort<int>("lockloss"),//判定锁定目标丢失时长毫秒,默认4000
            InputPort<int>("typ"),//控制类型，0：单机gps锁定，1：多机协同
            InputPort<int>("layidx")};  //处于分层中的索引};
        }

        CTraceStatus::CTraceStatus(const std::string & name,const BT::NodeConfig & config):
            SyncActionNode(name,config),m_state(Init),
            m_tsNow(InvalT){
            m_s_mixObj.preLoc = m_s_mixObj.curLoc = set::CSetTraceAttackObj::objLoc();//目标位置信息，地面发来的目标位置信息为gps坐标
            m_s_mixObj.ts=m_s_mixObj.preTs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            getInput<int>("mixloss",m_s_mixObj.len);//更新判定融合目标丢失时长
            getInput<int>("lockloss",m_track.len);//更新判定锁定目标丢失时长
            m_subObjRec = info::CParam::rosNode()->create_subscription<custom_msgs::msg::ObjectRecognition>(
            "inner/information/object_recognition",rclcpp::SensorDataQoS(),
            std::bind(&CTraceStatus::objRecCB,this,std::placeholders::_1));
            m_subObjMix = info::CParam::rosNode()->create_subscription<custom_msgs::msg::ObjectAttackDesignate>(
            "outer/set/attack_object_designate",rclcpp::SensorDataQoS(),
            std::bind(&CTraceStatus::objMixCB,this,std::placeholders::_1));
            m_subObjComp = info::CParam::rosNode()->create_subscription<custom_msgs::msg::ObjectComputation>(
            "inner/information/object_computation",rclcpp::SensorDataQoS(),
            std::bind(&CTraceStatus::objCompCB,this,std::placeholders::_1));
            m_typ = SingleLock;
            getInput<int>("typ",m_typ);
        }

        NodeStatus CTraceStatus::tick(){
            updateState();
            return NodeStatus::FAILURE;
        }
        
        void CTraceStatus::objRecCB(const custom_msgs::msg::ObjectRecognition::SharedPtr msg){
            for(auto it=msg->objs.begin();it!=msg->objs.end();++it){
                if(TrackId == it->id){
                    m_track.ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                }else if(m_yolo.id == it->id && (m_tsNow - m_yolo.ts < m_yolo.len)){
                    m_yolo.pixX = it->tpx + it->width/2;
                    m_yolo.pixY = it->tpy + it->height/2;
                }
            }
        }

        void CTraceStatus::objCompCB(const custom_msgs::msg::ObjectComputation::SharedPtr msg){
            int id = -1;//距离融合目标最近的yolo目标id
            float minDis = 1000*10;//距离融合目标最近的yolo目标距离
            for(auto it=msg->objs.begin();it!=msg->objs.end();++it){
                if(TrackId != it->id && CamGpsId != it->id){
                    float estx = m_s_mixObj.curLoc.x/1e3;
                    float esty = m_s_mixObj.curLoc.y/1e3;
                    //如当前没有融合目标但yolo和预测融合目标（依据速度）匹配则也认为匹配
                    if(m_tsNow - m_s_mixObj.ts > m_s_mixObj.len){
                        estx = (m_s_mixObj.curLoc.x + m_s_mixObj.spd.x*(m_tsNow-m_s_mixObj.ts))/1e3;
                        esty = (m_s_mixObj.curLoc.y + m_s_mixObj.spd.y*(m_tsNow-m_s_mixObj.ts))/1e3;                
                    }
                    float dis = algorithm::CUtility::getDisFrmLoc(estx,esty,
                    it->x/1e3,it->y/1e3);
                    if(dis < m_yolo.filtDis && dis < minDis){
                        minDis = dis;
                        id = it->id;
                    }
                }
            }
            if(id >= 0){
                m_yolo.id = id;
                m_yolo.ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            }
        }

        void CTraceStatus::objMixCB(const custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg){
            m_s_mixObj.ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            m_s_mixObj.curLoc = msg->objs;
            m_s_mixObj.objLat = msg->objs.x;
            m_s_mixObj.objLon = msg->objs.y;
            m_s_mixObj.objAlt = msg->objs.z;
            
            gps2loc(m_s_mixObj.objLat,m_s_mixObj.objLon,m_s_mixObj.objAlt,m_s_mixObj.curLoc.x,m_s_mixObj.curLoc.y,m_s_mixObj.curLoc.z);
            float len = m_s_mixObj.ts - m_s_mixObj.preTs;
            if(len > CompLen){
                m_s_mixObj.spd.x = (m_s_mixObj.curLoc.x-m_s_mixObj.preLoc.x)/(len);
                m_s_mixObj.spd.y = (m_s_mixObj.curLoc.y-m_s_mixObj.preLoc.y)/(len);
                m_s_mixObj.spd.z = (m_s_mixObj.curLoc.z-m_s_mixObj.preLoc.z)/(len);
                m_s_mixObj.preTs = m_s_mixObj.ts;
                m_s_mixObj.preLoc = m_s_mixObj.curLoc;
            }
        }

        void CTraceStatus::updateState(){
            //单机锁定时只发gps控制
            if(SingleLock == m_typ){
                sendCamAttiCtrl();
                return;
            }
            m_tsNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if((m_tsNow - m_track.ts) < m_track.len){
                m_state = Tracked;
                procTracked();
            }else if((m_tsNow - m_s_mixObj.ts) < m_s_mixObj.len){
                if((m_tsNow - m_yolo.ts) < m_yolo.len){
                    m_state = MixYolo;
                    procMixYolo();
                }else{
                    m_state = MixNoYolo;
                    procMixNoYolo();
                }
            }else{
                if((m_tsNow - m_yolo.ts) < m_yolo.len){
                    m_state = NoMixYolo;
                    procNoMixYolo();
                }else{
                    if(NoMixNoYolo != m_state){
                        /** 
                        setOutput<float>("objx",m_s_mixObj.curLoc.x/1e3);
                        setOutput<float>("objy",m_s_mixObj.curLoc.y/1e3);
                        setOutput<float>("objz",m_s_mixObj.curLoc.z/1e3);
                        setOutput<float>("objvx",m_s_mixObj.spd.x);
                        setOutput<float>("objvy",m_s_mixObj.spd.y);
                        */
                        //回中控制
                        setOutput<int>("ctrltyp",ctrl::CCameraCtrl::CtrlTyp::Cent);
                    }
                    m_state = NoMixNoYolo;
                }
                
            }
            setOutput<int>("state",m_state);
        }

        void CTraceStatus::procTracked(){
            sendOffbdCtrl(m_s_mixObj.curLoc.x/1e3,m_s_mixObj.curLoc.y/1e3);
            setOutput<int>("ctrltyp",ctrl::CCameraCtrl::CtrlTyp::Inval);
        }

        void CTraceStatus::procMixYolo(){
            sendOffbdCtrl(m_s_mixObj.curLoc.x/1e3,m_s_mixObj.curLoc.y/1e3);
            sendCamTrackCtrl();
        }

        void CTraceStatus::procMixNoYolo(){
            sendOffbdCtrl(m_s_mixObj.curLoc.x/1e3,m_s_mixObj.curLoc.y/1e3);
            sendCamAttiCtrl();
        }

        void CTraceStatus::procNoMixYolo(){
            float estx = (m_s_mixObj.curLoc.x + m_s_mixObj.spd.x*(m_tsNow-m_s_mixObj.ts))/1e3;
            float esty = (m_s_mixObj.curLoc.y + m_s_mixObj.spd.y*(m_tsNow-m_s_mixObj.ts))/1e3; 
            sendOffbdCtrl(estx,esty);
            sendCamTrackCtrl(); 
        }

        void CTraceStatus::sendOffbdCtrl(float objx,float objy){
            float z = -100;
            
            if(getInput<float>("tracez",z)){
                custom_msgs::msg::OffboardCtrl ctrl;
                ctrl.ordmask = EOffboardMask::LocCtrl + EOffboardMask::YawCtrl;
                if(get::CGetGroupLocation::isSelfFix()){
                    ctrl.ordmask = EOffboardMask::FixLocRadCtrl;
                    geometry_msgs::msg::Point32 pt;
                    
                    ctrl.vy = abs(z-m_s_mixObj.curLoc.z/1e3);//盘旋半径使用目标位置高度和飞机高度差
                }else{
                    ctrl.ordmask = EOffboardMask::LocCtrl+ EOffboardMask::YawCtrl;
                    ctrl.yaw = get::CGetGroupLocation::selfYaw();
                }
                ctrl.z = z;
                ctrl.x = objx;
                ctrl.y = objy;
                setOutput<custom_msgs::msg::OffboardCtrl>("target",ctrl);
            }
        }

        void CTraceStatus::sendCamAttiCtrl(){
            //原通过控制相机姿态进行现在直接用gps位置进行锁定控制
            setOutput<int>("ctrltyp",ctrl::CCameraCtrl::CtrlTyp::GpsTrack);
            /*geometry_msgs::msg::Point32 pt;
            get::CGetGroupLocation::getSelfLoc(pt);
            float yaw = algorithm::CUtility::getYawFrmLoc(pt.x,pt.y,m_s_mixObj.curLoc.x/1e3,
            m_s_mixObj.curLoc.y/1e3);
            geometry_msgs::msg::Point32 atti;
            get::CGetGroupLocation::getSelfAtti(atti);
            yaw = algorithm::CUtility::getRadDiff(atti.z,yaw);
            yaw = algorithm::CUtility::rad2ang(yaw);
            
            float pitch = algorithm::CUtility::getPitchDegFrmLoc(pt.x,pt.y,pt.z,
            m_s_mixObj.curLoc.x/1e3,m_s_mixObj.curLoc.y/1e3,m_s_mixObj.curLoc.z/1e3)-
            algorithm::CUtility::rad2ang(atti.y);
            */
            setOutput<int>("param1",m_s_mixObj.objLat);//lat,不使用yaw
            setOutput<int>("param2",m_s_mixObj.objLon);//lon,不使用pitch
            setOutput<int>("param3",m_s_mixObj.objAlt);//alt
            //std::cout<<"send lat:"<<m_s_mixObj.objLat<<" lon:"<<m_s_mixObj.objLon<< " alt:"<<m_s_mixObj.objAlt<<"\n";

        }

        void CTraceStatus::sendCamTrackCtrl(){
            setOutput<int>("ctrltyp",ctrl::CCameraCtrl::CtrlTyp::Track);
            setOutput<int>("param1",m_yolo.pixX);
            setOutput<int>("param2",m_yolo.pixY);
        }

        void CTraceStatus::gps2loc(int lat,int lon,float alt,int &x,int &y,int &z){
            geometry_msgs::msg::Point32 objGps;
            objGps.x = lat/1e7;//转为度
            objGps.y = lon/1e7;//转为度
            objGps.z = info::CParam::home().z - alt/1e3;//转为米

            algorithm::CUtility::gps2loc(info::CParam::home(),objGps);
            x = objGps.x*1e3;//mm
            y = objGps.y*1e3;//mm
            z = objGps.z*1e3;//mm
        }

    }
}