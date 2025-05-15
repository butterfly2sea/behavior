#include <takeoff_plugin.hpp>
#include <control/FlightmodeCtrl.hpp>
#include <interface.hpp>
namespace zyzn {
    namespace plugin {
        bool TakeoffPlugin::parseJsonParam(const Json::Value & params){
            try{
                for(const Json::Value & param : params){
                    if(param["name"].asString() == "alt") {
                        ctrl::CFlightmodeCtrl::takeoffZ() = getValue(param);
                        return true;
                    }
                }
            }
            catch(std::exception & e){
                return false;
            }
            return false;

        }

        std::shared_ptr<BasePlugin> create_plugin(){
            return std::make_shared<TakeoffPlugin>();
        }
        
    }
}