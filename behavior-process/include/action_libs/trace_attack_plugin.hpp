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
                POD_GPS_LOCK = 8,         ///<! ���ն���������gps����>
                CANCEL_POD_GPS_LOCK = 254 ///<! ����ȡ������������ȡ��gps����>
            };
            
            bool parseJsonParam(const Json::Value & params);

            void registerNode(BT::BehaviorTreeFactory &factory);
            
            bool procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,std::string & treeName);

        };

    }
}

#endif

