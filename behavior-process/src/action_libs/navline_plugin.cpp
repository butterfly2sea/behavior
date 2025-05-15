#include <stdexcept>
#include <navline_plugin.hpp>
#include <set/SetLine.hpp>
#include <utility/Utility.hpp>
#include <info/Param.hpp>
#include <check/CheckWayViaTp.hpp>
#include <interface.hpp>

namespace zyzn{
    namespace plugin{
        bool NavlinePlugin::parseJsonParam(const Json::Value & params){
            try{
                int type = Loc;
                set::CSetLine::loops() = 1;
                set::CSetLine::wayPts().points.clear();
                for(const Json::Value & param : params){
                        if (param["name"].asString() == "wayPoints") {
                        const Json::Value & v = param["value"];
                        if(!v.isArray())
                            return false;
                        set::CSetLine::wayPts().points.reserve(v.size());
                        for(const Json::Value & vv:v){
                            if(!vv.isObject())
                                return false;
                            if(vv.isMember("x_lat") && vv.isMember("y_lon") && vv.isMember("z_alt")){
                                geometry_msgs::msg::Point32 pt;
                                geometry_msgs::msg::Point ptd;
                                pt.x=ptd.x = vv["x_lat"].asDouble();
                                pt.y=ptd.y = vv["y_lon"].asDouble();
                                pt.z=ptd.z = vv["z_alt"].asDouble();
                                algorithm::CUtility::checkZValid<float>(pt.z);
                                ptd.z=pt.z;
                                set::CSetLine::wayPts().points.push_back(pt);                
                            }else{
                                return false;
                            }                   
                        }
                    }else if(param["name"].asString() == "loops"){
                        set::CSetLine::loops() = plugin::BasePlugin::getValue(param["value"]);

                    }else if(param["name"].asString() == "vehiType"){
                        if(param["value"].asString() == "多旋翼")
                            set::CSetLine::vehiType() = set::CSetLine::Coper;
                        else
                            set::CSetLine::vehiType() = set::CSetLine::FixWing;
                    }else if(param["name"].asString() == "pointTag"){
                        if(param.isMember("value"))
                            type = param["value"].asString()=="loc"?Loc:Gps;
                    }else if(param["name"].asString() == "spd"){
                        set::CSetLine::spd() = plugin::BasePlugin::getValue(param["value"]);
                        algorithm::CUtility::getPositiveVal<float>(set::CSetLine::spd());
                    }else if(param["name"].asString() == "arvDis"){
                        set::CSetLine::arvDis() = plugin::BasePlugin::getValue(param["value"]);
                        algorithm::CUtility::getPositiveVal<float>(set::CSetLine::arvDis());
                    }
                }
                if(Gps == type){
                    //gps转换为loc
                    algorithm::CUtility::gps2loc(info::CParam::m_sHome,set::CSetLine::wayPts().points);
                }
                return true;
            }
            catch(std::exception & e){
                return false;
            }
        }

        void NavlinePlugin::registerNode(BT::BehaviorTreeFactory &factory){
            factory.registerNodeType<check::CCheckWayViaTp>("CheckWayViaTp");
        }

        std::shared_ptr<BasePlugin> create_plugin(){
            return std::make_shared<NavlinePlugin>();
        }
        
    }
}