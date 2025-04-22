#ifndef ZYZN_CHECK_CHECKALLARRIVEDST_HPP
#define ZYZN_CHECK_CHECKALLARRIVEDST_HPP

#include <map>
#include <math.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <control/OffboardCtrl.hpp>
#include <info/OffboardInfo.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "../get/GetLocation.hpp"
#include "../utility/Utility.hpp"


using namespace BT;
namespace zyzn{
    namespace check{

        /**
         * @brief 检查是否全部到达目标
         * @details 现检测同分组内（航线飞行时）全部飞机朝向都指向航线的第一个航点
         * @author zyzn
         */  
        class CCheckAllArriveDst : public BT::SyncActionNode{
            public:

            /**
             * 飞机航向控制信息，包括当前航向和目标航向（弧度）
            */
            struct SCtrlYawInfo
            {
                float curYaw;//当前偏航
                float dstYaw;//目的偏航
                SCtrlYawInfo(){
                    curYaw = InitYaw;
                    dstYaw = InitYaw;
                }
            };                   
           
            /** 
             * @brief 获取输入、输出参数
             * info：本机信息
             * delay: 全部飞机航向就绪后的等待时间
             * type：飞机类型（1：旋翼，2：固定翼）
             * target:控制时的目的信息
             * @return 输入、输出参数列表
            */
            static PortsList providedPorts();

            
            CCheckAllArriveDst(const std::string& name, const NodeConfig& config);

            /**
             * @brief 节点定时回调 用于航线飞行，旋翼转固定翼前的控制（分组全部飞机指向第一个航点）依次判定本机是否到达目的航向、
             *          同组内全部飞机是否到达目的航向
             *          当全部飞机到达目的航向后，进入延迟倒计时，倒计时结束后返回成功；如已经是固定翼则直接返回成功
             * @return 
            */
            NodeStatus tick();

            private:

             /**
             * @brief 判定本机是否到达目的航向 依据航向偏差进行判定
             * @param yawDiff out 距离目标航向偏差值
             * @result 是否到达，true：是，false：否
            */
            bool isSelfInYaw(float &yawDiff);

            /**
             * @brief 分组内飞机是否全部到达目的航向 依据航向偏差进行判定，判定每个飞机
             * @result 是否到达，true：是，false：否
            */
            bool isAllOutInYaw();

            /**
             * @brief 是否依据到达目的航向 依据航向偏差进行判定
             * @param yawDiff in 距离目标航向偏差值
             * @result 是否到达，true：是，false：否
            */
            inline bool isInYaw(const float & yawDiff){
                if(1e3*abs(yawDiff) <= YawErrRad){
                    return true;
                }
                return false;
            }

            /**
             * @brief 判定飞机是否设置了目的航向 依据信息中是否包含该飞机及信息中的目的航向是否为初始值进行判定
             * @param id in 飞机id
             * @result 是否已经设置目的航向，true：是，false：否
            */
            bool isSetDstYaw(const uint8_t & id);

            /**
             * @brief 计算目的航向 依据飞机当前位置同目的位置（航线第一个航点和队形偏移值和）的偏航值
             * @param id in 飞机id
             * @param curx in 飞机当前位置locx
             * @param cury in 飞机当前位置locy
             * @result 偏航值
            */
            float getDstYaw(const uint8_t & id,float curx,float cury);

            /**
             * @brief 设置目的航向值 设置对应飞机的目的航向
             * @param id in 飞机id
             * @param yaw in 目的航向
             * @result 
            */
            inline void setDstYaw(const uint8_t & id,const float & yaw){
                m_info[id].dstYaw = yaw;
            }
            
            /**
             * @brief 更新控制航向 如距离目的航向偏差值小于到航向判定值则使用目的航向，否则在现航向基础上+偏差值/5
             * @param yawDiff in 距离目标航向偏差值
             * @result 
            */
            void updateCtrlYaw(const float & yawDiff);

            /**
             * @brief 外部飞机信息回调 收到飞机消息后依据是否在同分组进行处理，更新此飞机当前航向，并设置目的航向
             * @param msg 它机精简飞控消息
             * @result 
            */
            void outSimpleVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);
            
            /**
             * @brief 初始化分组航向信息 m_info中key只包含同分组内id信息
             * @result 
            */
            void initInfo();
        private:
            custom_msgs::msg::OffboardCtrl m_target;//控制信息
            std::map<int,SCtrlYawInfo> m_info;//分组内全部飞机的航向信息
            rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr m_subOutVehi;//外部飞机位置订阅对象
            std::chrono::system_clock::time_point m_tp;//倒计时间开始时间戳
            std::chrono::nanoseconds m_delay;          //延时长度,默认10秒 
        };
    }
}
#endif


