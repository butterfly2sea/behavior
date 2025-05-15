#ifndef ZYZN_PLUGIN_NAVLINE_PLUGIN_HPP
#define ZYZN_PLUGIN_NAVLINE_PLUGIN_HPP
#include <base_plugin.hpp>

namespace zyzn{
    namespace plugin{
        /**
         * @brief 
         */
        class NavlinePlugin: public BasePlugin{
            public:
            
            bool parseJsonParam(const Json::Value &params);

            void registerNode(BT::BehaviorTreeFactory &factory);

        };

    }
}

#endif

