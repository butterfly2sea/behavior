#ifndef ZYZN_CONTROL_TASKDECISION_HPP
#define ZYZN_CONTROL_TASKDECISION_HPP
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <custom_msgs/msg/command_request.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <custom_msgs/msg/object_attack_designate.hpp>
#include <custom_msgs/msg/image_distribute.hpp>
#include <custom_msgs/msg/param_short.hpp>
#include <custom_msgs/srv/command_bool.hpp>
#include <custom_msgs/srv/command_string.hpp>
#include <custom_msgs/msg/multispectral_cam_ctrl.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int32.hpp>

#include <manage/action_plugin_mgr.hpp>
#include <manage/ActionsMgr.hpp>
#include <set/SetLine.hpp>
#include <control/FlightmodeCtrl.hpp>
#include <get/GetLocation.hpp>
#include <status/Weapon.hpp>

namespace zyzn{
    
    namespace ctrl{
        class CTaskDecision : public rclcpp::Node,std::enable_shared_from_this<CTaskDecision>
        {
            public:
                /**
                 * Ros话题类型
                */
                enum ETopicType{

                    SetImgDistri=0,   ///<!图片分发设置>
                    SetImgChan,       ///<!图片通道设置>
                    SetHome,          ///<!home点设置>
                    CmdReq,           ///<!控制指令请求>
                    TaskSet,          ///<!结构体任务设置>
                    AttackDesn,       ///<!攻击任务>
                    TaskStatus,       ///<!任务状态>
                    CmdRes,           ///<!控制指令回复>
                    GetSelfId,        ///<!获取本机id>
                    SetJsonStage,     ///<!json任务设置>
                    GetRtspUrl,       ///<!获取rtsp地址>
                    ReportAutoLocLine,///<!上报自动生成航线>
                    SetVehiType,      ///<!设置载具类型（旋翼、固定翼）>
                    CtrlWeapon,       ///<!武器、物资控制>
                    Multispectral,    ///<!多光谱控制>
                    TopicCount
                };

                enum ETopicQueSize{
                    ShortSz=10,
                    LongSz=50
                };

                /**
                 * id标识
                */
                enum EVehiFlag{
                    GrdControl=0,      ///<!地面站>
                    AllVehi=0xFF       ///<!任意飞机，全部飞机>
                };

                typedef void (f_nxtTask_t) (void);
                typedef boost::signals2::signal<f_nxtTask_t> sig_nxtTask_t;
                typedef status::CCommandStatus::ECmd ECmd;
                typedef status::CCommandStatus::ECmdStatus ECmdStatus;


                sig_nxtTask_t sig_nxtTask;
                CTaskDecision(const char* nodeName,std::shared_ptr<manage::CActionsMgr>actMgr,
                std::shared_ptr<manage::ActionPluginMgr> plgMgr);

                /**
                 * @brief 包括话题名、qos及节点id的设置及注册
                 * @param path 程序路径
                 * @result
                */
                void regisAllNode(const std::string & path);

                /**
                 * @brief 编队节点内部会需要持续更新消息话题，而节点本身ros node只支持tick时才更新话题，所以其它
                 * 需要持续更新的使用此函数参数的ros node
                 * @param node in out 持续更新消息的Sub、Pub或serice用到的ros node
                */
                void setNode(rclcpp::Node::SharedPtr node);

                static CTaskDecision* ins();
                
                /**
                 * @brief 设置home点,利用对应的home点设置发布对象进行消息发布
                 * @param lat in home点纬度 deg*1e7
                 * @param lon in home点经度 deg*1e7
                 * @param alt in home点海拔高度 m*1e3（mm）
                */
                void setHome(int32_t lat,int32_t lon,int32_t alt);


                private:
                /**
                 * @brief 指定打击目标回调,收到打击指定目标时将此目标信息保存
                 * @param msg in 打击的目标信息
                */
                void attackDesnCallback(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg);
                
                /**
                 * @brief 控制指令回调,针对不同的控制指令进行处理，并回复对应消息
                 * @param msg in 指令信息
                */
                void commandCallback(const custom_msgs::msg::CommandRequest::SharedPtr msg);


                /**
                 * @brief 阶段任务回调,目前只处理分组设置和偏移设置
                 * @param msg in 阶段任务信息
                */
                void formResetCB(const custom_msgs::msg::TaskStage::SharedPtr msg);


                /**
                 * @brief 初始化 创建全部订阅、发布对象
                */
                void init();
                
                /**
                 * @brief 利用system 将收到的系统时间UTC（ms，共8字节），低4字节为lwt 高4字节为hgt设置为本机系统时间
                 * @param lwt in 时间的低4字节
                 * @param hgt in 时间的高4字节
                */
                void asyntime(int32_t lwt,int32_t hgt);

                /**
                 * @brief 设置图片分发,利用对应的图片分发发布对象进行消息发布
                 * @param typ in 分发类型 0：ros话题，1：udp，2：rtsp,3:停止分发
                 * @param rcvip in 图片接收ip（udp方式时使用）
                 * @param rcvport in 图片接收方端口（udp方式时使用）
                 * @param resx in 分辨率 长度值
                 * @param resy in 分辨率 宽度值
                 * @param fps in 帧率
                */
                void setImgDistribute(uint8_t typ,uint32_t rcvip,uint16_t rcvport,uint16_t resx,uint16_t resy,uint8_t fps);
                
                /**
                 * @brief 图片通道切换,利用对应的图片通道发布对象进行消息发布，当ch为3时表示设置多光谱通道
                 * @param ch in 通道号
                 * @param mult in 多光谱通道号
                 * @param photographed in 是否开启拍照
                */
                void setImgChannel(uint8_t ch,uint8_t mult,uint8_t photographed);

                /**
                 * @brief 获取rtsp地址 通过服务从其它节点获取rtsp地址
                 * @param url in out rtsp地址
                */
                bool getRtspUrl(std::string &url);

                private:
                void joyCB(const sensor_msgs::msg::Joy::SharedPtr joy);
                bool filterCmd(const custom_msgs::msg::CommandRequest::SharedPtr & msg);
               
                /**
                 * @brief 初始化请求回复
                */
                void initResp(const custom_msgs::msg::CommandRequest::SharedPtr & msg);
                
                /**
                 * @brief 处理视频请求命令
                */
                void procVideoCmd(const custom_msgs::msg::CommandRequest::SharedPtr & msg);


                /**
                 * @brief 是否需要脱离分组，当收到其它飞机需要执行的指令时，需要判定该飞机是否需要脱离当前执行的任务
                 * @param cmd 指令
                */
                inline bool needGetoffGroup(int cmd){
                    return (ECmd::Takeoff==cmd || ECmd::Land==cmd || ECmd::Return==cmd || ECmd::Loiter==cmd || ECmd::DoTask==cmd
                    || ECmd::Joystick==cmd || ECmd::DesignAttackObj==cmd);
                }
            
                private:
                rclcpp::Subscription<custom_msgs::msg::CommandRequest>::SharedPtr m_cmdReq;//控制指令订阅对象
                rclcpp::Subscription<custom_msgs::msg::TaskStage>::SharedPtr m_taskSet;//任务订阅对象
                rclcpp::Subscription<custom_msgs::msg::ObjectAttackDesignate>::SharedPtr m_objAttackDesig;//打击指定目标订阅对象
                rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_jsonTask;//json任务订阅对象
                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_subJoy;//摇杆数据订阅

                rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_setHome;//home点设置对象
                rclcpp::Publisher<custom_msgs::msg::ImageDistribute>::SharedPtr m_imgDistri;//图像分发发布对象
                rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_imgChan;//图像通道切换发布对象
                rclcpp::Publisher<custom_msgs::msg::MultispectralCamCtrl>::SharedPtr m_multispectralCtrl;//多光谱控制发布对象
                
                rclcpp::Client<custom_msgs::srv::CommandString>::SharedPtr m_getRtspUrl;//获取rtsp地址客户端

                
                rclcpp::Node::SharedPtr m_node;             //行为叶子节点中用到的ros2 node
                std::shared_ptr<manage::CActionsMgr> m_actMgr;//json解析对象
                std::shared_ptr<manage::ActionPluginMgr> m_pluginMgr;//行为树插件管理对象 处理行为树

                std::shared_ptr<set::CSetLine> m_setLine;        //航线设置对象
                std::shared_ptr<ctrl::CFlightmodeCtrl> m_flymdCtrl;//飞行模式设置
                std::shared_ptr<get::CGetlocation> m_getLoc;//获取飞机信息
                std::shared_ptr<status::Weapon> m_weapon;   //武器控制及信息获取
        };
    }
    

}

#endif