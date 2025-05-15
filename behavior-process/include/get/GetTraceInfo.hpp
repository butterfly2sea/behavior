#ifndef ZYZN_GET_TRACE_INFO_HPP
#define ZYZN_GET_TRACE_INFO_HPP
#include<set>
#include<behaviortree_cpp/action_node.h>
#include <behavior_lib/get/GetGroupLocation.hpp>
#include <behavior_lib/utility/Utility.hpp>
using namespace BT;
namespace zyzn{
    namespace get{
        /**
         * @brief 用于多机跟踪时获取跟踪高度、跟随飞机及分层索引值
         */
        class CTraceInfo : public SyncActionNode{
            public:
            enum EZTyp{
                SelfZ=0,
                LayerZ
            };

            CTraceInfo(const std::string &name,const BT::NodeConfig& config);
            static PortsList providedPorts();

            NodeStatus tick();
            private:

            /**
             * @brief 获取离z最近的飞机
             * @param z 目标高度
             * @param vehis 飞机集合
             * @return 返回飞机id
             */
            CGetGroupLocation::id_t getNearVehi(float z,const CGetGroupLocation::vehis_t &vehis);

            /**
             * @brief 计算多机跟踪跟踪高度、分层及跟随飞机信息
             */
            void compTraceInfo();

            private:
            std::vector<CGetGroupLocation::id_t> m_layer;//每分层对应的id
            std::set<CGetGroupLocation::id_t> m_exId;//查找距离某高度时排除的id即此id已经被找到过
        };
    }
}


#endif