#ifndef ZYZN_PLUGIN_TAKEOFF_PLUGIN_HPP
#define ZYZN_PLUGIN_TAKEOFF_PLUGIN_HPP
#include <base_plugin.hpp>

namespace zyzn{
    namespace plugin{
        /**
         * @brief 
         */
        class TakeoffPlugin: public BasePlugin{
            public:
            
            bool parseJsonParam(const Json::Value &params);


        };

    }
}

#endif

