#ifndef ZYZN_GET_GROUP_LOCATION_HPP
#define ZYZN_GET_GROUP_LOCATION_HPP
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/point32.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <set/SetLine.hpp>

using namespace BT;
namespace zyzn{
    namespace get{
        /**
         * @brief 获取分组中飞机位置信息包括本机及它机
        */
        class CGetGroupLocation : public SyncActionNode{
            public:
            typedef u_int8_t id_t;
            typedef geometry_msgs::msg::Point32 loc_t;
            typedef float z_t;
            typedef std::map<id_t,custom_msgs::msg::SimpleVehicle> vehis_t;
            enum{
                Highest = -999999,
                InvalId = 0
            };

            static PortsList providedPorts()
            {
                return { OutputPort<float>("test")};//
            }

           
            CGetGroupLocation(const std::string & name,const BT::NodeConfig & config);
            
            NodeStatus tick();
            
            /**
             * @brief 是否已经获取到分组中全部飞机信息，已经分组中id信息和已经取得的飞机信息
             * @return 是否已经获取了全部信息
            */
            static bool hasAllVehis();
            
            /**
             * @brief 获取本机的loc信息
             * @param out loc 飞机loc值
             * @return 是否成功
            */
            static bool getSelfLoc(loc_t & loc);
            
            /**
             * @brief 获取指定id飞机的loc信息
             * @param in id 飞机id
             * @param out loc 飞机loc值
             * @return 是否成功
            */
            static bool getLoc(id_t id,loc_t & loc);

            static bool getSelfAtti(loc_t & atti);

            static bool getAtti(id_t id,loc_t & atti);
            
             /**
             * @brief 获取分组中最低高度飞机id及高度
             * @param out id 飞机id
             * @param out z 最低高度
             * @return 是否成功
            */
            static bool getMinZ(id_t & id,z_t &z);

            /**
             * @brief 获取指定飞机信息 获取指定id飞机精简飞控信息
             * @param in id 飞机id
             * @param out val 返回值是否有效
             * @return 飞机信息
            */
            static const custom_msgs::msg::SimpleVehicle & getVehi(id_t id,bool & val);

            inline static const vehis_t & allVehis(){
                return m_vehis;
            }

            static bool isSelfFix();

            static float selfYaw();
            private:
            void innerVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);

            void outerVehiCB(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);
            void procMsg(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);


            private:
            rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr m_subSelfVehi;
            rclcpp::Subscription<custom_msgs::msg::SimpleVehicle>::SharedPtr m_subOutVehi;
            static vehis_t m_vehis;//全部飞机信息

        };

    }
}
#endif