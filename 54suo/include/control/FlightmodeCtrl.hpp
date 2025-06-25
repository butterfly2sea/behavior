#ifndef ZYZN_CONTROL_FLIGHTMODECTRL_HPP
#define ZYZN_CONTROL_FLIGHTMODECTRL_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/srv/command_long.hpp>
using namespace BT;
namespace zyzn{
    namespace ctrl{
        /**
         * @brief 飞行模式控制
         * @details 依据输入参数 设置飞行模式
         * @author zyzn
        */
        class CFlightmodeCtrl : public StatefulActionNode{
            public:
            /**
             * 飞行模式值
            */
            enum EFlightMode{
                Unknown=0,   //未知
                Manual,      //手动
                Alt,         //定高
                Pos,         //定点
                TakeOff,     //起飞
                Land,        //降落
                Hold,        //等待
                Mission,     //任务
                Rtl,         //返航
                Offboard,    //外部
                Stable       //增稳
            };
            CFlightmodeCtrl(const std::string & instance_name,
            const BT::NodeConfig& conf);
            
            CFlightmodeCtrl();

            /**
             * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
             * mode：期望的飞行模式
             * param7：起飞模式时的起飞高度
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();
            

            /**
             * @brief 必须重载父类函数,状态为初始时调用,判定服务是否可用，如可用返回RUNNING态
             * @return RUNNING:服务可用 IDEL:服务不可用
            */
            NodeStatus onStart();
            
            /**
             * @brief 必须重载父类函数,状态为RUNNING时调用,
             * 判定服务是否已经为期望状态，如未达到则每隔10tick发送一次请求
             * @return RUNNING:未达到状态 SUCCESS:达到状态
            */
            NodeStatus onRunning();

            /**
             * @brief 必须重载父类函数,结束时调用,当状态为RUNNING 框架调用halt时的回调
             * @return 无
            */
            void onHalted();

             /**
             * @brief 更新飞行模式，提供给非行为树节点调用，主要响应降落、返航模式
             * @param md 期望飞行模式
             * @return 无
            */
            void updateMode(int md);

            /**
             * @brief 设置起飞高度，现用offboard起飞时使用的z
             * @param z 期望高度
             * @return 无
            */
            inline void setTakeoffZ(float z){
                m_s_takeoffZ = z;
            }

            /**
             * @brief 重置tick计数，外部模式控制时需先重置计数
             * @return 无
            */
            inline void resetTick(){
                m_tickCount = 0;
            }

            inline static float & takeoffZ(){
                return m_s_takeoffZ;
            }
            private:
            void init();
            private:
            static rclcpp::Client<custom_msgs::srv::CommandLong>::SharedPtr m_s_cltFlymd;//锁控制对象
            int m_tickCount;//tick计数,用于每隔几次tick发送一次服务
            int m_expMode;//期望模式状态用于保存输入参数值
            static float m_s_takeoffZ;//起飞高度
        };
    }

}

#endif