#ifndef ZYZN_PLUGIN_DEST_SEARCH_PLUGIN_HPP
#define ZYZN_PLUGIN_DEST_SEARCH_PLUGIN_HPP
#include <base_plugin.hpp>

namespace zyzn{
    namespace plugin{

        class DestSearchPlugin: public BasePlugin{
            public:
            
            bool parseJsonParam(const Json::Value &params);

            void registerNode(BT::BehaviorTreeFactory &factory);
            
            bool procCmd(const custom_msgs::msg::CommandRequest::SharedPtr msg,std::string & treeName);


        };

    }
}
#endif