#ifndef ZYZN_STATUS_TRACE_STATUS_HPP
#define ZYZN_STATUS_TRACE_STATUS_HPP
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/object_recognition.hpp>
#include <custom_msgs/msg/object_attack_designate.hpp>
#include <geometry_msgs/msg/point32.hpp>

namespace zyzn{
    namespace status{
        using namespace BT;
        /**
         * @brief 多机跟踪状态机处理类
         */
        class CTraceStatus : public BT::SyncActionNode{
            public:
            typedef int64_t ts_t;
            enum EState{
                Tracked=0,   ///<! 锁定状态>
                MixYolo,     ///<! 未锁定+融合目标+yolo>
                MixNoYolo,   ///<! 未锁定+融合目标+无yolo>
                NoMixYolo,   ///<! 未锁定+无融合目标+yolo>
                NoMixNoYolo, ///<! 未锁定+无融合目标+无yolo>
                Init=-1
            };

            enum{
                InvalT = 0,     ///<! 无效时间>
                DefLen = 4000,  ///<! 默认失效时长>
                TrackId = 1,    ///<! 锁定目标id>
                CamGpsId = 2,   ///<! 吊舱自己解算的gps目标>
                DefFiltDis = 40, ///<! 默认目标过滤距离>
                CompLen=1000,
                SingleLock = 0,  ///<! 单机锁定>
                MultiLock = 1  ///<! 多机锁定>
            };
            
            //融合目标速度估算相关
            struct SMixInfo{
                SMixInfo():preTs(InvalT),
                ts(InvalT),
                len(DefLen){
                }
                ts_t preTs;//计算融合目标速度时上次目标信息的保存时间
                ts_t ts;//当前融合目标收到时间ms
                custom_msgs::msg::ObjectLocation preLoc;//上次目标位置信息
                geometry_msgs::msg::Point32 spd;//估算速度
                custom_msgs::msg::ObjectLocation curLoc;//当前融合目标信息
                int objLat;//当前目标的gps lat 位置
                int objLon;//当前目标的gps lon 位置
                int objAlt;//当前目标的gps alt 位置
                int len;//判定融合丢失时长ms
                  
            };

            //
            struct STrackInfo{
                STrackInfo():ts(InvalT),
                len(DefLen){
                }
                ts_t ts;//上次锁定框收到时间ms
                int len;//判定锁定丢失时长ms
            };

            struct SMatchYolo{
                SMatchYolo():pixX(0),
                pixY(0),
                id(0),
                ts(InvalT),
                len(DefLen),
                filtDis(DefFiltDis){
                }
                int pixX;//box中心x
                int pixY;//box中心y
                uint32_t id;//目标id
                ts_t ts;//上次yolo匹配时间ms
                int len;//判定yolo丢失时长ms
                float filtDis;//目标过滤距离判定融合目标和yolo目标是否为相同目标的距离
            };

            /**
             * @brief 输入输出端口
             */
            static PortsList providedPorts();

            CTraceStatus(const std::string & name,const BT::NodeConfig & config);
            
            /**
             * @brief 节点定时执行函数,由行为树调度
             * @return 总是返回FAILURE
             */
            NodeStatus tick();

            inline static SMixInfo & mixObj(){
                return m_s_mixObj;
            }

            /**
             * @brief gps转loc
             * @param lat 纬度 deg*1e7
             * @param lon 经度 deg*1e7
             * @param alt 高度 mm
             * @param x x坐标 mm
             * @param y y坐标 mm
             * @param z z坐标 mm
             */
            static void gps2loc(int lat,int lon,float alt,int &x,int &y,int &z);

            private:

            /**
             * @brief 目标识别结果（box框）回调函数
             * @param msg in 识别消息
             */
            void objRecCB(const custom_msgs::msg::ObjectRecognition::SharedPtr msg);

            /**
             * @brief 目标位置结算结果回调函数
             * @param msg in 位置结算消息
             */
            void objCompCB(const custom_msgs::msg::ObjectComputation::SharedPtr msg);

            /**
             * @brief 目标融合结果（由地面发来）回调函数
             * @param msg in 融合目标消息
             */
            void objMixCB(const custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg);
            
            /**
             * @brief 更新状态
             */  
            void updateState();

            /**
             * @brief 处理跟踪状态
             */
            void procTracked();

            /**
             * @brief 处理有融合目标有yolo目标状态
             */
            void procMixYolo();

            /**
             * @brief 处理有融合目标无yolo目标状态
             */
            void procMixNoYolo();

            /**
             * @brief 处理无融合目标有yolo目标状态
             */
            void procNoMixYolo();

            /**
             * @brief 发送位置控制指令
             * @param objx 目标x坐标
             * @param objy 目标y坐标
             */ 
            void sendOffbdCtrl(float objx,float objy);

            /**
             * @brief 发送相机姿态控制指令
             */
            void sendCamAttiCtrl();

            /**
             * @brief 发送相机锁定控制指令
             */
            void sendCamTrackCtrl();


            private:
            rclcpp::Subscription<custom_msgs::msg::ObjectRecognition>::SharedPtr m_subObjRec;//目标识别
            rclcpp::Subscription<custom_msgs::msg::ObjectAttackDesignate>::SharedPtr m_subObjMix;//融合目标
            rclcpp::Subscription<custom_msgs::msg::ObjectComputation>::SharedPtr m_subObjComp;//目标位置解算
            static SMixInfo m_s_mixObj;//存放融合目标相关信息
            SMatchYolo m_yolo;//存放匹配yolo信息
            STrackInfo m_track;//存放锁定相关信息
            EState m_state;//当前跟踪状态
            ts_t m_tsNow;
            int m_typ;//控制类型 0:单机gps锁定，1：多机锁定控制
            
        };

    }
}

#endif 