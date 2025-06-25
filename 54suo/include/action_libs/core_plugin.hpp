#ifndef ZYZN_PLUGIN_CORE_PLUGIN_HPP
#define ZYZN_PLUGIN_CORE_PLUGIN_HPP
#include <base_plugin.hpp>

namespace zyzn{
    namespace plugin{
        /**
         * @brief 实现核心插件，包括飞行模式设置、offboard控制、位置获取、航线相关设置、控制切换遥杆控制等基本行为树节点
         * 其它插件会用到此插件中的部分功能
         */
        class CorePlugin: public BasePlugin{
            public:
            
            bool parseJsonParam(const Json::Value &params);

            void registerNode(BT::BehaviorTreeFactory &factory);
            
            bool procCmd(const custom_msgs::msg::CommandRequest::SharedPtr msg,std::string & treeName);

            bool procJoyCtrl(std::string & treeName);

        };

    }
}

#endif

