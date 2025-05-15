#ifndef ZYZN_STATUS_WEAPON_HPP
#define ZYZN_STATUS_WEAPON_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "CommandStatus.hpp"

namespace zyzn{
    namespace status{
        enum EDevStatus{
            OPEN=0,       ///<! 打开>
            INFO_ERROR,   ///<! 设备信息错误>
            NO_AUTHORITY, ///<! 无权限>
            UNKNOWN       ///<! 未知>
        };

        
        class Weapon{
            public:
            enum{
                WEAPON_FUN_NUM=100,///<! 武器控制功能总数>
                UNLOCKER=1,        ///<! 解保>
                TRIGGER=2,         ///<! 引爆>
                DEV_TYPE=8,        ///<! 设备类型>
                CMD_TYPE=20        ///<! 指令类型>
            };

            Weapon();

            /**
             * @brief 武器（物资）控制,利用对应的武器控制发布对象进行消息发布
             * @param enable in 控制类型 1：使能，0：取消
             * @param cmd 指令 1:上电 2:解保 3:引爆 4:物资投放
            */
            void weaponCtrl(int enable,int cmd);

            /**
             * @brief 解保状态
             * @return 解保:1,未解保:0
            */
            inline int unlockerSts(){
                return (UNLOCKER & m_sts);
            }

            /**
             * @brief 引爆状态
             * @return 引爆:1,未引爆:0
            */
            int triggerSts(){
                return (TRIGGER & m_sts);
            }

            /**
             * @brief 状态回复
            */
            void respSts();

            private:
            void statusCallback(const std_msgs::msg::UInt8::SharedPtr sts);

            private:
            rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  m_subSts;//武器状态订阅
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_pubCtrl;//控制发布
            int m_sts;//武器状态
            CCommandStatus m_cmdResp;//指令回复对象

        };
    }
}




#endif
