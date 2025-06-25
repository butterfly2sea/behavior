#include<map>
#include<set>
#include<rclcpp/rclcpp.hpp>
#include <action_libs/base_plugin.hpp>
#include"../../include/manage/ActionsMgr.hpp"
#include"../../include/info/Param.hpp"
#include"../../include/manage/SpiralLineGen.hpp"
#include"../../include/utility/Utility.hpp"
#include"../../include/set/SetLine.hpp"
#include "../../include/set/SetTraceAttackObj.hpp"
#include "../../include/set/SetDstPt.hpp"
#include "../../include/control/FlightmodeCtrl.hpp"

namespace zyzn{
    namespace manage{

        CActionsMgr::CActionsMgr(){
 
        }
        
        CActionsMgr::~CActionsMgr(){

        }

        bool CActionsMgr::getFormoffset(int grpId,const Json::Value & actions,std::vector<geometry_msgs::msg::Point32> & rslt){
            rslt.clear();
            for (const Json::Value & action:actions){
                if(action.isMember("groupid") && action["groupid"].asInt() == grpId){
                    const Json::Value & params = action["params"];
                    if(!params.isArray())
                        continue;                   
                    for(const Json::Value & param:params){
                        if(!(param.isMember("name") && param["name"].asString() == "formOffset"))
                            continue;
                        const Json::Value & vv = param["value"];
                        if(vv.isObject() && vv.isMember("diff_x_lat") && vv.isMember("diff_y_lon") && vv.isMember("diff_z_alt")){
                            geometry_msgs::msg::Point32 pt;
                            pt.x = vv["diff_x_lat"].asFloat();
                            pt.y = vv["diff_y_lon"].asFloat();
                            pt.z = vv["diff_z_alt"].asFloat();
                            rslt.push_back(pt);
                        }                       
                    }                   
                }
            }
            return rslt.size();           
        }

        void CActionsMgr::processStage(const Json::Value &stage ){
            if(stage.isMember("name") && stage.isMember("sn") && stage.isMember("cmd") && stage.isMember("actions")) { 
                std::string cmd(stage["cmd"].asCString());
                if("del" == cmd ||"set" == cmd  || "ins" == cmd){
                    return;
                }                             
                const Json::Value & actions = stage["actions"];
                RCLCPP_INFO(rclcpp::get_logger("ActrionMgr"),"actions size:%d",actions.size());
                if(!actions.isArray())
                    return;
                Json::Value dstAct(Json::objectValue);
                std::map<int,std::vector<int>> grpInfos;
                std::set<uint8_t> stIds;//执行开始任务的飞机id集合
                int selfGrpId = -1;
                for(const Json::Value & action:actions){
                    if(action.isMember("name") && action.isMember("id") && action.isMember("groupid")){
                        int grpid = plugin::BasePlugin::getValue(action["groupid"]);
                        int id = plugin::BasePlugin::getValue(action["id"]);
                        if("start" == cmd)
                            stIds.insert(id);//只添加开始任务的飞机
                        if(std::find(grpInfos[grpid].begin(),grpInfos[grpid].end(),id) == grpInfos[grpid].end())
                            grpInfos[grpid].push_back(id);
                        if(id != info::CParam::m_sSelfId)
                            continue;
                        RCLCPP_INFO(rclcpp::get_logger("ActrionMgr"),"action:%s id:%d grpid:%d",
                         action["name"].asString().c_str(),id,grpid);
                        selfGrpId = grpid;
                        if(action.isMember("params")){
                            dstAct["params"] = action["params"];
                        }
                        if(action.isMember("triggers")){
                            dstAct["triggers"] = action["triggers"];
                        }
                        dstAct["name"] = action["name"];
                    }
                }
                if(selfGrpId != -1){//包括本机执行任务
                    set::CSetLine::grp() = selfGrpId;
                    set::CSetLine::clearExtIds();
                    set::CSetLine::ids().assign(grpInfos[selfGrpId].begin(),grpInfos[selfGrpId].end());
                    getFormoffset(selfGrpId,actions,set::CSetLine::offsets().points);
                    sig_procJsonAct(dstAct,cmd.c_str(),stage["sn"].asInt());
                }else if(!stIds.empty()){//不为本机执行任务，需要把开始执行任务的飞机进行分组排除
                    set::CSetLine::ids_t exIds;
                    exIds.assign(stIds.begin(),stIds.end());
                    sig_setExtIds(exIds);
                }
            }           
        }

        void CActionsMgr::jsonTaskCallback(const std_msgs::msg::String::SharedPtr msg){
            Json::Value rot;
            Json::CharReaderBuilder builder;
            std::istringstream ss(msg.get()->data);
            Json::String err;
            Json::parseFromStream(builder, ss, &rot,&err);
            if(rot.isMember("stage")){
                const Json::Value & stages = rot["stage"];
                if(stages.isArray() && !stages.empty()){
                    for(const Json::Value & stage:stages){
                        processStage(stage);
                    }
                }else{
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"stage is not array or size is 0");
                }
            }
        }      

 
    }
}