
#include <sys/time.h>
#include <dm/bytes.hpp>
#include <time.h>
#include <stdexcept>

#include <behaviortree_cpp/bt_factory.h>

#include <control/TaskDecision.hpp>
#include <get/GetLocation.hpp>
#include <info/Param.hpp>
#include <utility/Utility.hpp>
#include <status/CommandStatus.hpp>
#include <control/TraceAttackCtrl.hpp>
#include <control/LockCtrl.hpp>

#include <action_libs/core_plugin.hpp>

#include <chrono>
using namespace std::chrono_literals;
static zyzn::ctrl::CTaskDecision* scurIns = nullptr;
namespace zyzn{
    namespace ctrl{
        rclcpp::SensorDataQoS sesQs;
        rclcpp::QoS qs1(1);
        
        static const char* topicNames[CTaskDecision::ETopicType::TopicCount] ={
            "inner/set/image_distribute",
            "inner/set/image_channel",
            "inner/set/coord_calibration",
            "outer/command/request",
            "outer/set/task_stage",
            "outer/set/attack_object_designate",
            "outer/information/status_task",
            "outer/command/response",
            "inner/get/id_vehicle",
            "outer/set/stage_info_json",
            "inner/get/rtsp_url",
            "outer/information/auto_navline",
            "inner/set/vehicle_type",
            "inner/control/weapon",
            "/multispectral_camera/control"
        };

        static rclcpp::QoS *topicQos[CTaskDecision::ETopicType::TopicCount];

        
        
        CTaskDecision::CTaskDecision(const char* nodeName) : rclcpp::Node(nodeName),
        m_cmdReq(nullptr),//
        m_taskSet(nullptr),
        m_objAttackDesig(nullptr),
        m_setHome(nullptr),
        m_imgDistri(nullptr),
        m_imgChan(nullptr),
        m_multispectralCtrl(nullptr),
        m_getRtspUrl(nullptr),
        m_node(nullptr),
        m_actMgr(nullptr),
        m_pluginMgr(nullptr),
        m_setLine(nullptr),
        m_flymdCtrl(nullptr),m_getLoc(nullptr),m_weapon(nullptr){
            init();           
        }

        void CTaskDecision::attackDesnCallback(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg){
             bool atc = false;
            for(int i=0;i<msg.get()->ids.size();++i){
                RCLCPP_INFO(get_logger(),"id[%d]=%d",i,(int)(msg.get()->ids[i]));
                if(msg.get()->ids[i] == info::CParam::m_sSelfId){
                    atc = true;
                    break;
                }
            }
            
            if(!atc){
                RCLCPP_INFO(get_logger(),"ignore attack or trace order");
                //它机跟踪打击任务(除去终止任务)时 重新规划分组及分组偏移
                if(ctrl::CTraceAttackCtrl::Abort != msg->type){
                    m_setLine->setExIds(msg->ids);
                }
                return;
            }
            
            //RCLCPP_INFO(get_logger(),"rcv attack obj id:%d x:%d y:%d z:%d",msg.get()->objs.id,msg.get()->objs.x,msg->objs.y,msg->objs.z);   
            m_pluginMgr->procTraceAttack(msg,status::CCommandStatus::cmdRspMsg().rslt);
        }

        void CTaskDecision::initResp(const custom_msgs::msg::CommandRequest::SharedPtr & msg){
            status::CCommandStatus::cmdRspMsg().id = info::CParam::m_sSelfId;
            status::CCommandStatus::cmdRspMsg().src = 0;
            status::CCommandStatus::cmdRspMsg().type = msg.get()->type;
            status::CCommandStatus::cmdRspMsg().rslt = "";
            status::CCommandStatus::cmdRspMsg().status = ECmdStatus::Failed;
        }

        void CTaskDecision::procVideoCmd(const custom_msgs::msg::CommandRequest::SharedPtr & msg){
            RCLCPP_INFO(get_logger(), "set video param0:%d",msg.get()->param0);
            setImgDistribute(msg.get()->param0,msg.get()->param1,msg.get()->param2,msg.get()->param3,msg.get()->param4,msg.get()->fparam5);
            if(msg.get()->param0 == 2){
                if(getRtspUrl(status::CCommandStatus::cmdRspMsg().rslt)){
                    status::CCommandStatus::cmdRspMsg().status = status::CCommandStatus::Success;
                }
            }
            status::CCommandStatus::sendCmdRsp();  
        }

        void CTaskDecision::commandCallback(const custom_msgs::msg::CommandRequest::SharedPtr msg){
            if(!filterCmd(msg))
                return;            
            status::CCommandStatus::initResp(msg->type,status::CCommandStatus::ECtrlType::Cmd);
            if(msg.get()->type == status::CCommandStatus::ECmd::SetVideo){
                procVideoCmd(msg);
                return;
            }
            status::CCommandStatus::cmdRspMsg().status = status::CCommandStatus::Success;
            switch(msg.get()->type){
                case ECmd::Land:
                    m_flymdCtrl->updateMode(ctrl::CFlightmodeCtrl::Land);
                    break;
                case ECmd::Return:
                    m_flymdCtrl->updateMode(ctrl::CFlightmodeCtrl::Rtl);
                    break;
                case ECmd::CmdSetHome:
                    setHome(msg.get()->param1,msg.get()->param2,msg.get()->param3);
                    break;
                case ECmd::SyncTime:
                    asyntime(msg.get()->param0,msg.get()->param1);
                    break;
                case ECmd::SetVehi:                                                           
                    m_setLine->setVehiTyp(msg.get()->param0);
                    break;
                case ECmd::SetImgCh:
                    setImgChannel(msg.get()->param0,msg.get()->param1,msg.get()->param2);
                    break;
                case ECmd::Weapon:
                    m_weapon->weaponCtrl(msg.get()->param0,msg.get()->param1);
                    break;
                default:
                    if(!m_pluginMgr->procCmd(msg,status::CCommandStatus::cmdRspMsg().rslt))
                        status::CCommandStatus::cmdRspMsg().status = status::CCommandStatus::Failed;
                    break;
            }

            status::CCommandStatus::sendCmdRsp();
            
        }


        void CTaskDecision::joyCB(const sensor_msgs::msg::Joy::SharedPtr joy){
            m_pluginMgr->procJoy(status::CCommandStatus::cmdRspMsg().rslt);
        }

        bool CTaskDecision::filterCmd(const custom_msgs::msg::CommandRequest::SharedPtr & msg){
            if((msg.get()->type == 254) ){//ignore heartbeat
                return false;
            }
            RCLCPP_INFO(get_logger(), "cmd recv type: %d param0:%d param1:%d param2:%d param3:%d param4:%d dstid:%d", msg.get()->type,
            msg->param0,msg->param1,msg->param2,msg->param3,msg->param4,msg->dst);
            if(msg.get()->dst != info::CParam::m_sSelfId ){
                RCLCPP_INFO(get_logger(), "cmd ignore selfid: %d dstid: %d", info::CParam::m_sSelfId,msg.get()->dst);
                //为其它飞机执行任务时需要进行脱离分组操作
                if(needGetoffGroup(msg->type)){
                    set::CSetLine::ids_t ids;
                    ids.push_back(msg->dst);
                    m_setLine->setExIds(ids);
                }
                return false;
            }
            return true;
        }

        //重新设置编队偏移
        void CTaskDecision::formResetCB(const custom_msgs::msg::TaskStage::SharedPtr msg){
            if(set::CSetLine::isInGroup(msg->head.ids) ){//如本机处于分组中则更新分组及偏移
                if(msg->head.ids.size()==msg->formoffset.points.size()){
                    m_setLine->updateIds(msg->head.ids);
                    m_setLine->updateOffsets(msg->formoffset);
                    if(set::CSetLine::isSwitchSetLine()){
                        m_setLine->setWayPts(info::CParam::m_sCurStage.line);
                    }
                    if(set::CSetLine::isSwitchSetLoops()){
                        m_setLine->setLoops(m_setLine->loops());
                    }
                    RCLCPP_INFO(get_logger(),"update ids and offsets");
                }              
            }else{//如本机不处于分组中则从原来分组中排除此分组飞机
                m_setLine->setExIds(msg->head.ids);  
            }
        }
             

        void CTaskDecision::init(){

            declare_parameter<int>("switchSets",set::CSetLine::TWO_SWITCH);//分组编队变化时的设置量
            get_parameter("switchSets",set::CSetLine::switchSets());
            RCLCPP_INFO(get_logger(),"switchSets:%d",set::CSetLine::switchSets());
            m_actMgr = std::make_shared<manage::CActionsMgr>();

            info::CParam::m_sHome.z = algorithm::CUtility::EDefVal::InvalidAlt;
            for (size_t i = 0; i < TopicCount; i++){
                topicQos[i] = &qs1;
            }
            //更新qos
            topicQos[SetHome] = &sesQs;
            
            m_imgDistri = this->create_publisher<custom_msgs::msg::ImageDistribute>(topicNames[ETopicType::SetImgDistri],ShortSz);
            m_imgChan = this->create_publisher<std_msgs::msg::UInt8>(topicNames[ETopicType::SetImgChan],ShortSz);
            m_multispectralCtrl = this->create_publisher<custom_msgs::msg::MultispectralCamCtrl>(topicNames[ETopicType::Multispectral],ShortSz);
            m_setHome = this->create_publisher<geometry_msgs::msg::Point>(topicNames[ETopicType::SetHome],ShortSz);
           
            m_jsonTask = this->create_subscription<std_msgs::msg::String>(topicNames[ETopicType::SetJsonStage],ShortSz,
            std::bind(&manage::CActionsMgr::jsonTaskCallback,m_actMgr.get(),std::placeholders::_1));

            m_cmdReq = this->create_subscription<custom_msgs::msg::CommandRequest>(topicNames[ETopicType::CmdReq],ShortSz,
            std::bind(&CTaskDecision::commandCallback,this,std::placeholders::_1));
            m_taskSet = this->create_subscription<custom_msgs::msg::TaskStage>(topicNames[ETopicType::TaskSet],ShortSz,
            std::bind(&CTaskDecision::formResetCB,this,std::placeholders::_1));
            m_objAttackDesig = this->create_subscription<custom_msgs::msg::ObjectAttackDesignate>(topicNames[ETopicType::AttackDesn],ShortSz,
            std::bind(&CTaskDecision::attackDesnCallback,this,std::placeholders::_1));
            scurIns =this; 
        }
        void CTaskDecision::setNode(rclcpp::Node::SharedPtr node){
            m_node = node;
            m_getRtspUrl = m_node->create_client<custom_msgs::srv::CommandString>(topicNames[ETopicType::GetRtspUrl]);
        }

         CTaskDecision* CTaskDecision::ins(){
            return scurIns;
        }

        bool CTaskDecision::getRtspUrl(std::string &url){
            if(!m_getRtspUrl.get()->wait_for_service(1s)){
                RCLCPP_WARN(get_logger(), "getRtspUrl service not available now");
                return false;
            }
            auto req = std::make_shared<custom_msgs::srv::CommandString::Request>();
            std::shared_future<std::shared_ptr<custom_msgs::srv::CommandString::Response>> result = m_getRtspUrl.get()->async_send_request(req);
            
            if(rclcpp::spin_until_future_complete(m_node, result) == rclcpp::FutureReturnCode::SUCCESS){
                url = result.get().get()->rslt;
                RCLCPP_INFO(get_logger(), "getRtspUrl: %s", url.c_str());
                return true;
            }else{
                RCLCPP_WARN(get_logger(), "Failed to call service getRtspUrl");
                return false;
            }
        }

        void CTaskDecision::regisAllNode(const std::string & path){
     
            m_setLine = std::make_shared<set::CSetLine>();
            m_flymdCtrl = std::make_shared<ctrl::CFlightmodeCtrl>();
            m_getLoc = std::make_shared<get::CGetlocation>();

            m_weapon = std::make_shared<status::Weapon>();
            m_actMgr->sig_setExtIds.connect(boost::bind(&set::CSetLine::setExIds, m_setLine,boost::placeholders::_1)); 

            m_pluginMgr = std::make_shared<manage::ActionPluginMgr>();
            m_actMgr->sig_procJsonAct.connect(boost::bind(&manage::ActionPluginMgr::procJsonAction,m_pluginMgr,
            boost::placeholders::_1,boost::placeholders::_2,boost::placeholders::_3));

            m_pluginMgr->loadPlugin(path,"config.json");
        }

        void CTaskDecision::asyntime(int32_t lwt,int32_t hgt){
            dm::int64 ms = dm::bytes2int64(dm::lowByte_i32(lwt),dm::lowByte_i32(lwt,1),dm::lowByte_i32(lwt,2),dm::lowByte_i32(lwt,3),
                                           dm::lowByte_i32(hgt),dm::lowByte_i32(hgt,1),dm::lowByte_i32(hgt,2),dm::lowByte_i32(hgt,3));
     
            double timestamp_ = double(ms)/1000.0;
            char command[128]={0};
            //3588:marvsmart orinNx:nvidia
            sprintf(command, "echo '%s'|sudo -S date -s @%.9f",std::getenv("USER"), timestamp_);
            RCLCPP_INFO(get_logger(),"synctime 命令是:%s",command);
            // 执行命令
            if (system(command) != 0) {
                RCLCPP_WARN(get_logger(),"Failed to set system time.");
            } else {
                RCLCPP_INFO(get_logger(),"System time has been set.");
            }
        }

        void CTaskDecision::setImgDistribute(uint8_t typ,uint32_t rcvip,uint16_t rcvport,uint16_t resx,uint16_t resy,uint8_t fps){
            custom_msgs::msg::ImageDistribute imgDis;
            imgDis.type = typ;
            imgDis.rcvip = rcvip;
            imgDis.rcvport = rcvport;
            imgDis.resx = resx;
            imgDis.resy = resy;
            imgDis.fps = fps;
            if(m_imgDistri.get())
                m_imgDistri.get()->publish(imgDis);

        }

        void CTaskDecision::setImgChannel(uint8_t ch,uint8_t mult,uint8_t photographed){
            if (m_multispectralCtrl.get() && m_imgChan.get())
            {
                RCLCPP_INFO(get_logger(), "set img ch %d",ch);
                custom_msgs::msg::MultispectralCamCtrl multCtrl;
                if (ch == 3) {
                    multCtrl.enable_camera = 1;
                    multCtrl.switch_channel = mult;
                    multCtrl.photographed = photographed;
                    m_multispectralCtrl.get()->publish(multCtrl);
                }
                else if (ch < 3)
                {
                    multCtrl.enable_camera = 0;
                    m_multispectralCtrl.get()->publish(multCtrl);

                    std_msgs::msg::UInt8 channel;
                    channel.data = ch;
                    m_imgChan.get()->publish(channel);
                }  
            }  
        }

        void CTaskDecision::setHome(int32_t lat,int32_t lon,int32_t alt){
            if(m_setHome){
                if(ctrl::CLockCtrl::Unlock == get::CGetlocation::simpVehi().lock){
                    RCLCPP_WARN(get_logger(),"当前飞机解锁状态不支持设置home点");
                    status::CCommandStatus::cmdRspMsg().rslt = "当前飞机解锁状态不支持设置home点";
                    return;
                }             
                info::CParam::m_sHome.x = lat/1e7;
                info::CParam::m_sHome.y = lon/1e7;
                info::CParam::m_sHome.z = alt/1e3;
                m_setHome.get()->publish(info::CParam::m_sHome);
                RCLCPP_INFO(get_logger(), "set home lat:%d lon:%d alt:%d",lat,lon,alt);
            }
        }

    }
    
}