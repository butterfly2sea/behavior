#ifndef ZYZN_PLUGIN_TRACEATTACK_PLUGIN_HPP
#define ZYZN_PLUGIN_TRACEATTACK_PLUGIN_HPP
#include <plugin/base_plugin.hpp>

namespace zyzn{
    namespace plugin{
        /**
         * @brief 
         */
        class TraceAttackPlugin: public BasePlugin{
            public:

            void registerNode(BT::BehaviorTreeFactory &factory);
            
            bool procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,std::string & treeName);

        };

    }
}

#endif

