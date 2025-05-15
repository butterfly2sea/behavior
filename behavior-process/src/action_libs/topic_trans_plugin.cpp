#include <topic_trans_plugin.hpp>
#include <interface.hpp>
#include <control/TopicTrans.hpp>
namespace zyzn {
    namespace plugin {
        bool TopicTransPlugin::parseJsonParam(const Json::Value & params){
            try{
                for(const Json::Value & param : params){
                    if(param["name"].asString() == "transValue") {
                        ctrl::TopicTrans::info() = param["value"].asCString();
                        return true;
                    }
                }
            }
            catch(std::exception & e){
                return false;
            }
            return false;

        }

        void TopicTransPlugin::registerNode(BT::BehaviorTreeFactory &factory){
            factory.registerNodeType<ctrl::TopicTrans>("TopicTrans");
        }

        std::shared_ptr<BasePlugin> create_plugin(){
            return std::make_shared<TopicTransPlugin>();
        }
        
    }
}