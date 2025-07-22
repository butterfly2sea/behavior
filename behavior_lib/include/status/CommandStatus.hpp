#ifndef ZYZN_STATUS_COMMAND_HPP
#define ZYZN_STATUS_COMMAND_HPP

#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/command_response.hpp>
#include <custom_msgs/msg/status_task.hpp>

using namespace BT;
namespace zyzn{
    namespace status{
        /**
         * @brief 用于使用行为节点发布指令回复信息，包括命令回复和任务状态回复
        */
        class CCommandStatus : public SyncActionNode{
            public:

            /**
             * 命令控制指令类型
            */
            enum ECmd{
            Test=0,              //测试 
            Takeoff=1,	         //起飞
            Land=2,	             // 降落
            Loiter=3,	         // 盘旋
            Return=4,            // 返航


            Point=5,             //指点飞行

            CmdSetHome=6,        // 对准坐标原点
            SetTargetLOc=7,      //设置目标位置信息
            DoTask=8,            //执行任务
            SetVideo=9,          //图片视频指令
            GetFtpInfo=10,       //获取文件传输信息
            GetID=11,            //

            SyncTime=12,         //时间同步


            SendHeartBeat=14,    //发送心跳
            SetVehi=15,          ///<!设置载具类型>
            SetImgCh=19,         ///<!设置图片通道>
            Weapon=21,           ///<!武器控制>
            Joystick=23,         ///<! 摇杆控制>
            SetStage=128,        ///<!>
            DesignAttackObj=129  ///<!打击指定目标>

            };
            

            /**
             * 任务状态
            */
            enum EStatusStg{
                StsNoStart,///<!未开始>
                StsOngoing,///<!正在进行>
                StsFailed, ///<!失败>
                StsComplete,///<!完成>
                StsNotready,///<!未就绪>
                StsNull
            };

            /**
             * 指令回复中cmd 类型值
            */
            enum ECtrlType{
                Cmd=0,        ///<!回复的是控制指令结果>
                TraceAttack,  ///<!回复的是跟踪、打击结果>
                JsonTask      ///<!json任务>
            };

            /**
             * 返回的控制指令执行状态
            */
            enum ECmdStatus{
                Success=0,      ///<!成功>
                Failed=1,       ///<!失败>
            };

            /**
             * @brief 使用的上报类型
            */
            enum ERspType{
                CMD_RSP=1,  ///<! 指令状态上报>
                TASK_RSP=2, ///<! 任务状态上报>
                BOTH_RSP=CMD_RSP|TASK_RSP ///<! 指令状态+任务状态上报>
            };


            CCommandStatus(const std::string& name,
            const NodeConfig& conf);

            CCommandStatus();
            /**
             * @brief 输入输出参数，节点提供的输入输出参数名及类型申明
             * cmd：消息对应的cmd参数
             * subcmd：消息对应的subcmd参数
             * status：消息对应的status参数
             * rslt：消息对应的rslt参数
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();

            static void sendCmdRsp();


            static void sendTaskStatus();
            
            static inline custom_msgs::msg::StatusTask & stsTskMsg(){
                return m_s_msgStsTsk;
            }

            static inline custom_msgs::msg::CommandResponse & cmdRspMsg(){
                return m_s_msgCmdRsp;
            }

            static void initResp(int subCmd,ECtrlType type=Cmd);

            /**
             * @brief 定时回调,发送回复信息
             * @result 总是返回SUCCESS
            */
            NodeStatus tick();   

            private:
            void init(); 
            private:
            static rclcpp::Publisher<custom_msgs::msg::CommandResponse>::SharedPtr m_s_pubResp;
            static rclcpp::Publisher<custom_msgs::msg::StatusTask>::SharedPtr m_s_pubStsTask;//任务状态上报发布对象
            static custom_msgs::msg::StatusTask m_s_msgStsTsk;//任务状态上报消息
            static custom_msgs::msg::CommandResponse m_s_msgCmdRsp;//消息回复对象
            int m_rspType;//上报类型
        };
    }
}

#endif