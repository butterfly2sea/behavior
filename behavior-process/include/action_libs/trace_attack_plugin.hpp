#ifndef ZYZN_PLUGIN_TRACEATTACK_PLUGIN_HPP
#define ZYZN_PLUGIN_TRACEATTACK_PLUGIN_HPP
#include <base_plugin.hpp>

namespace zyzn{
    namespace plugin{
        /**
         * @brief 
         */
        class TraceAttackPlugin: public BasePlugin{
            public:
            enum TraceAttackCtrl{
                POD_GPS_LOCK = 8,         ///<! 吊舱二次锁定即gps锁定>
                CANCEL_POD_GPS_LOCK = 254 ///<! 吊舱取消二次锁定即取消gps锁定>
            };
            
            bool parseJsonParam(const Json::Value & params);

            void registerNode(BT::BehaviorTreeFactory &factory);
            
            bool procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,std::string & treeName);

        };

    }
}

#endif

