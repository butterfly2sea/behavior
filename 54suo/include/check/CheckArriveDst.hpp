#ifndef ZYZN_CHECK_CHECKARRIVEDIS_HPP
#define ZYZN_CHECK_CHECKARRIVEDIS_HPP

#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/simple_vehicle.hpp>

using namespace BT;
namespace zyzn{
    namespace check{
        /**
         * @brief 检查是否到达目标点，可包括只判定垂直向是否到达；三向距离是否到达；其它飞机是否到达
        */
        class CCheckArriveDst : public SyncActionNode{
        public:
        CCheckArriveDst(const std::string& name,const NodeConfig& conf);


         /**
         * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
         * arvDis：到点距离 米
         * target：目的点信息
         * onlyz： 只判定垂向距离
         * @result 输入输出参数列表
        */
        static PortsList providedPorts();

        /**
        * @brief 定时回调函数，通过对比本机当前位置和目标点距离判定是否到达目的点
        * @return SUCCESS：已经到达，FAILED：未到达
        */
        NodeStatus tick();
        };
}

}

#endif