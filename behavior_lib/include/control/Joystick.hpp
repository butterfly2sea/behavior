#ifndef ZYZN_CONTROL_JOYSTICK
#define ZYZN_CONTROL_JOYSTICK
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <mavros_msgs/msg/manual_control.hpp>
#include <sensor_msgs/msg/joy.hpp>


using namespace BT;
namespace zyzn{
    namespace ctrl{
        /**
         * @brief 摇杆控制，处理地面发来的摇杆数据并转换为mavros的mannual 控制消息进行转发
         * 当规定的时间内没收到地面发来的数据时，认为地面摇杆控制结束，退出摇杆控制
        */
         class CJoystick : public BT::SyncActionNode{
            public:
            enum {
                MinMan = -1000, ///<! 手动控制消息的最小值>
                MaxMan = 1000,  ///<! 手动控制消息的最大值>
                MinJoy = 1000,  ///<! 摇杆消息的最小值>
                MaxJoy = 2000,  ///<! 摇杆消息的最大值>
                MidJoy = 1500,  ///<! 摇杆消息的中值>
                PitchCh = 1,    ///<! 俯仰通道索引>
                RollCh = 0,     ///<! 横滚通道索引>
                ThroCh = 2,     ///<! 油门通道索引>
                YawCh = 3,      ///<! 偏航通道索引>
                InvalTs = 0,    ///<! 无限时间值>
                LostCtrlLen = 2000///<! 失去摇杆数据时长判定 毫秒>
            };
            CJoystick(const std::string& name, const NodeConfig& config);
            
            /**
             * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
             * lost：失去摇杆控制时长判定，毫秒
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();

            /**
             * @brief 定时执行函数，会由父节点调用
             * @return SUCCESS：将摇杆转换手动消息进行手动控制，FAILED：摇杆消息丢失停止控制
            */
            NodeStatus tick();

            private:

            /**
             * @brief 定时执行函数，会由父节点调用，将摇杆消息转换为手动控制消息内容
             * @param joy 摇杆消息
            */
            void joyCB(const sensor_msgs::msg::Joy::SharedPtr joy);

             /**
             * @brief 将摇杆值转换为手动控制值，从一个范围映射到另个范围
             * @param j in 摇杆值
             * @param m out 手动值
            */
            inline void joy2Man(const int & j,float & m){
                m = (j-MidJoy)*2;
                if(m < MinMan)
                    m = MinMan;
                else if(m > MaxMan)
                    m = MaxMan;
            }
            public:
            static bool m_s_startJoy;//开启摇杆
            private:
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_subJoy;//摇杆数据接收对象，由地面站来经DDS转发
            rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr m_pubManCtrl;//mavros手动控制消息发布对象
            mavros_msgs::msg::ManualControl m_ctrl;//mavros手动控制消息
            int64_t m_rcvTs;//记录上次收到摇杆数据的时间 毫秒
            int m_lostLen;//判定失去摇杆控制的时间
            int m_pitchCh;//俯仰通道索引
            int m_rollCh;//横滚通道索引
            int m_throtCh;//油门通道索引
            int m_yawCh;//偏航通道索引
         };
    }
}

#endif