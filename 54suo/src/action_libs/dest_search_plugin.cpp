#include <dest_search_plugin.hpp>
#include <utility/Utility.hpp>
#include <set/SetDstPt.hpp>
#include <info/Param.hpp>
#include <status/CommandStatus.hpp>
#include <interface.hpp>
namespace zyzn{
    namespace plugin{
        bool DestSearchPlugin::parseJsonParam(const Json::Value & params){
            try{
                int type = Loc;
                bool isValid = false;//判断参数是否有效现只判定目的点
                for(const Json::Value & param : params){
                    if(param["name"].asString() == "radius"){
                        set::CSetDstPt::dstPt().rdis = getValue(param);
                    }else if(param["name"].asString() == "intval"){
                        set::CSetDstPt::dstPt().intval= getValue(param);
                    }else if(param["name"].asString() == "alt"){
                        set::CSetDstPt::dstPt().alt = getValue(param)*-1;
                    }
                    else if(param["name"].asString() == "pointTag"){
                        if(param.isMember("value"))
                            type = param["value"].asString()=="loc"?Loc:Gps;
                    }else if(param["name"].asString() == "dstLoc"){
                        if(param.isMember("value")){
                            const Json::Value & val = param["value"];
                            if(val.isArray() && val.size()>0){
                                const Json::Value & pt = val[0];
                                if(pt.isMember("x_lat"))
                                    set::CSetDstPt::dstPt().dstPt.x = getValue(pt["x_lat"]);   
                                if(pt.isMember("y_lon"))
                                    set::CSetDstPt::dstPt().dstPt.y = getValue(pt["y_lon"]);
                                if(pt.isMember("z_alt"))
                                    set::CSetDstPt::dstPt().dstPt.z = getValue(pt["z_alt"]);
                                algorithm::CUtility::checkZValid<float>(set::CSetDstPt::dstPt().dstPt.z);    
                                isValid = true;                
                            }                    
                        }                
                    }                           
                }
                if(Gps == type){
                    algorithm::CUtility::gps2loc(info::CParam::m_sHome,set::CSetDstPt::dstPt().dstPt);
                }
                return isValid;
            }
            catch(std::exception & e){
                return false;
            }
            return false;
        }

        void DestSearchPlugin::registerNode(BT::BehaviorTreeFactory &factory) {
            factory.registerNodeType<set::CSetDstPt>("SetDstPt");
        }

        bool DestSearchPlugin::procCmd(const custom_msgs::msg::CommandRequest::SharedPtr msg,std::string & treeName){
            if(status::CCommandStatus::ECmd::Point == msg->type){
                return true;
            }
            return false;
        }

        std::shared_ptr<BasePlugin> create_plugin(){
            return std::make_shared<DestSearchPlugin>();
        }
    }
}