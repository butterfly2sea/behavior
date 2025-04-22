#ifndef ZYZN_CONTROL_LOCALROUTE_HPP
#define ZYZN_CONTROL_LOCALROUTE_HPP
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/bt_factory.h"
#include <geometry_msgs/msg/polygon.hpp>

#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/srv/get_dis_target.hpp>
#include <custom_msgs/srv/command_int.hpp>

using namespace BT;

namespace zyzn
{

    namespace ctrl
    {
        /**
         * @brief loc航点控制
         * @details 开始集群编队节点延迟联调，原用此来替代集群编队节点，后弃用
         * @author zyzn
        */
        class CRouteLocal : public SyncActionNode
        {
            typedef geometry_msgs::msg::Polygon WpList_t;
            typedef geometry_msgs::msg::Point32 SPos;
            /**
             * 集群编队控制指令
            */
            enum ECtrlMode{
                Start=0,  ///<!开始>
                Pause,    ///<!暂停>
                Continue, ///<!继续>
                Stop,     ///<!停止>
                NullMode  ///<!无效指令>
            };
        
        public:
            CRouteLocal(const std::string& name,
            const NodeConfig& conf);

            /**
             * @brief 输入输出参数
             * @details 节点提供的输入输出参数名及类型申明
             * ctrl：offboard控制量信息，由其它节点提供值
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();


            /**
             * @brief 设置航线,更新航点信息，并重置当前航点idx，领航点及目标点信息
             * @param line in 全部航点信息
             * @result
            */
            void setLine(const geometry_msgs::msg::Polygon &line);

            /**
             * @brief 发送控制信息,将领航点信息更新到offboard控制消息内容
             * @result
            */
            NodeStatus tick();

            /**
             * @brief 创建全局的订阅及服务对象,
             * 树节点只会在tick时才会更新自生消息，但简单飞控消息、航点服务则需要被其它节点使用
             * 故需要持续更新消息，所以创建这些订阅及服务使用的rosnode 是外部的，而非树节点自生的
             * @param msg in out offboard控制指令
             * @result
            */
            void setNode(rclcpp::Node::SharedPtr nd);
    

        private:
            /**
             * @brief 是接近（到达）目标点 依据当前点距离目标点距离（0.3）进行判定
             * @result true：到到，false：未到达
            */
            bool isNearPilot();
            /**
             * @brief 是接近（到达）领航点,依据当前点距离领航点距离（0.3）进行判定
             * @result true：到到，false：未到达
            */
            bool isNearMid();

            /**
             * @brief 如已经到达目标点则更新为下个航点
             * 如已经接近当前领航点，则更新领航点
             * @result 
            */
            void updatePilot();

            /**
             * @brief 更新下个航点,如航点已经完成，则更新当前航点为第一个航点，
             * 依据当前航点和下个航点计算步长，（会利用步长更新领航点）
             * @result 
            */
            void toNextWp();

            /**
             * @brief 精简飞控消息回调,利用消息，更新当前位置
             * @param msg in 精简飞控消息
             * @result 
            */
            void simpleVehicleCallback(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);

            /**
             * @brief 航点信息获取回调,计算当前位置距离目标航点的距离、当前目标航点的索引（0开始）
             * @param request 服务请求
             * @param response 服务返回结果
             * @result 
            */
            void getDisTargetCallback(const custom_msgs::srv::GetDisTarget::Request::SharedPtr request,
                                      custom_msgs::srv::GetDisTarget::Response::SharedPtr response);

            /**
             * @brief 编队控制切换,更新当前的控制指令，并返回控制结果
             * @param request 服务请求
             * @param response 服务返回结果
             * @result 
            */
            void ctrSwitchCallback(const custom_msgs::srv::CommandInt::Request::SharedPtr request,  
                                      custom_msgs::srv::CommandInt::Response::SharedPtr response);
        
        private:
            
            rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr m_subSimpVehi;//精简飞控订阅对象
            rclcpp::Service<custom_msgs::srv::GetDisTarget>::SharedPtr m_srvGetDisTarget;  //航点信息服务对象
            rclcpp::Service<custom_msgs::srv::CommandInt>::SharedPtr   m_srvCtrSwitch;     //编队控制服务对象
            WpList_t m_wpItems; //航点信息列表
            SPos m_posPilot;    //领航点，offboard控制 loc 位置
            SPos m_curPos;      //当前位置信息
            SPos m_tgtPos;      //目标航点位置信息
            bool m_isCollect;   //
            float m_segs;       //当前航段内分割的数量
            float m_curSeg;     //当前航段分割数索引
            float m_diffx;      //位置更新步长x量
            float m_diffy;      //位置更新步长y量
            float m_diffz;      //位置更新步长z量
            int m_curIdx;       //航点索引
            int m_ctrlMode;     //控制指令
            rclcpp::Node::SharedPtr m_node; //创建需要一直更新消息、服务的rosnode

        };
                     
    } // namespace 
} // namespace zyzn




#endif