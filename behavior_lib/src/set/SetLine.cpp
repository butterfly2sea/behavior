#include <rclcpp/rclcpp.hpp>
#include <log/Logger.hpp>
#include "../../include/set/SetLine.hpp"
#include "../../include/info/Param.hpp"
#include "../../include/utility/Utility.hpp"

namespace zyzn{
    namespace set{
        //节点的inputs端口名，用于参数名获取
        static const char * g_inputNames[CSetLine::JsonParamIdx::PARAMS_COUNT]={
            "vehiTypParam","spdParam","ptTypParam","disParam","ptsParam","lpsParam"
        };
        //默认json任务中参数名
        static const char * g_defParams[CSetLine::JsonParamIdx::PARAMS_COUNT]={
            "vehiType","spd","pointTag","arvDis","wayPoints","loops"
        };
        int CSetLine::m_s_loops = 1;
        int CSetLine::m_s_vehiType = CSetLine::Coper;
        float CSetLine::m_s_arvDis = 0.5;
        int CSetLine::m_s_switchSets = CSetLine::SetContentTyp::TWO_SWITCH;
        std::set<uint8_t> CSetLine::m_s_extIds;
        uint8_t CSetLine::m_s_grp;
        CSetLine::ids_t CSetLine::m_s_ids;
        geometry_msgs::msg::Polygon CSetLine::m_s_offsets;
        geometry_msgs::msg::Polygon CSetLine::m_s_wayPts;
        float CSetLine::m_s_spd=1;

        CSetLine::CSetLine(const std::string & name,
        const BT::NodeConfig& conf):SyncActionNode(name,conf){
            int typ = ALL;
            getInput<int>("type",typ);
            m_setTyp = (SetContentTyp)typ;
            m_s_loops = 1;//默认循环次数
            updateParam();
            init();
        }
        CSetLine::CSetLine():SyncActionNode("",BT::NodeConfig()){
            m_setTyp = ALL;
            init();
        }


        PortsList CSetLine::providedPorts(){
            return {
            InputPort<float>("antiDis"), //避撞距离
            InputPort<int>("type"),      //设置内容类型
            InputPort<std::string>(g_inputNames[JsonParamIdx::WAY_PTS_IDX]),//json参数中航点参数名
            InputPort<std::string>(g_inputNames[JsonParamIdx::LOOPS_IDX]),//json参数中循环次数参数名
            InputPort<std::string>(g_inputNames[JsonParamIdx::VEHICLE_TYP_IDX]),//json参数中载具参数名
            InputPort<std::string>(g_inputNames[JsonParamIdx::PT_TYP_IDX]),//json参数中航点类型参数名
            InputPort<std::string>(g_inputNames[JsonParamIdx::SPD_IDX]),//json参数中速度参数名
            InputPort<std::string>(g_inputNames[JsonParamIdx::ARV_DIS_IDX])//json参数中到点距离参数名
            };
        }

        NodeStatus CSetLine::tick(){
            float dis = 1;
            getInput<float>("antiDis", dis);
            setAntiCollDis(dis);
            setArvDis(m_s_arvDis);
            //setForm(info::CParam::m_sCurStage.form);
            setGrp(m_s_grp);
            setIds(m_s_ids);
            setOffsets(m_s_offsets);
            setWayPts(m_s_wayPts);
            setLoops(m_s_loops);
            setSpd(m_s_spd);
            setVehiTyp(m_s_vehiType);
            return NodeStatus::SUCCESS;
        }

        void CSetLine::init(){
            m_pubTime = nullptr;
            m_pubLine = nullptr;
            m_pubCollDis = nullptr;
            m_pubArvDis = nullptr;
            m_pubGrp = nullptr;
            m_pubLoops = nullptr;
            m_pubSpd = nullptr;
            m_pubVehiTyp = nullptr;
            m_pubIds = nullptr;
            m_pubOffsets = nullptr;
            m_pubTime = info::CParam::rosNode()->create_publisher<std_msgs::msg::Int64>(
                "inner/set/form_time",1);
            if(WAY_PTS&m_setTyp)
                m_pubLine = info::CParam::rosNode()->create_publisher<geometry_msgs::msg::Polygon>(
                "inner/set/navline",1);
            if(ANTI_DIS&m_setTyp)
                m_pubCollDis = info::CParam::rosNode()->create_publisher<std_msgs::msg::Float32>(
                "inner/set/dis_anti_collide",1);
            if(ARV_DIS&m_setTyp)
                m_pubArvDis = info::CParam::rosNode()->create_publisher<std_msgs::msg::Float32>(
                "inner/set/dis_arrive",1);
            if(FORM&m_setTyp)
                m_pubForm = info::CParam::rosNode()->create_publisher<custom_msgs::msg::ParamShort>(
                "inner/set/form",1);
            if(GROUP&m_setTyp)
                m_pubGrp = info::CParam::rosNode()->create_publisher<std_msgs::msg::UInt8>(
                "inner/set/group",1);
            if(LOOPS&m_setTyp)
                m_pubLoops = info::CParam::rosNode()->create_publisher<std_msgs::msg::Int32>(
                "inner/set/line_loops",1);
            if(SPD&m_setTyp)
                m_pubSpd = info::CParam::rosNode()->create_publisher<std_msgs::msg::Float32>(
                "inner/set/form_spd",1);
            if(VEHICLE_TYP&m_setTyp)
                m_pubVehiTyp = info::CParam::rosNode()->create_publisher<std_msgs::msg::UInt8>(
                "inner/set/vehicle_type",1);
            if(IDS&m_setTyp)
                m_pubIds = info::CParam::rosNode()->create_publisher<std_msgs::msg::UInt8MultiArray>
            ("inner/information/group_ids",1);
            if(OFFSETS&m_setTyp)
                m_pubOffsets = info::CParam::rosNode()->create_publisher<geometry_msgs::msg::Polygon>
            ("inner/information/group_offset",1);
        }

        void CSetLine::setAntiCollDis(float dis){
            if(m_pubCollDis){
                std_msgs::msg::Float32 msg;
                msg.data = dis;
                m_pubCollDis->publish(msg);
            }           
        }

        void CSetLine::setArvDis(float dis){
            if(m_pubArvDis){
                std_msgs::msg::Float32 msg;
                msg.data = dis;
                m_pubArvDis->publish(msg);
                txtLog().info(THISMODULE "setArvDis:%f",msg.data);
            }           
        }

        void CSetLine::setForm(const custom_msgs::msg::ParamShort& typ){
            if(m_pubForm){
                m_pubForm->publish(typ);
                txtLog().info(THISMODULE "setForm:%d",typ.type);
            }
            
        }

        void CSetLine::setGrp(uint8_t grp){
            if(m_pubGrp){
                std_msgs::msg::UInt8 msg;
                msg.data = grp;
                m_pubGrp->publish(msg);
                txtLog().info(THISMODULE "setGrp:%d",msg.data);
            }
            
        }

        void CSetLine::setWayPts(const geometry_msgs::msg::Polygon& pts){
            if(m_pubLine){
                m_pubLine->publish(pts);
                txtLog().info(THISMODULE "setline pts number:%d",pts.points.size());
            }
        }

        void CSetLine::setLoops(int lp){
            if(m_pubLoops){
                std_msgs::msg::Int32 msg;
                msg.data = lp;
                m_pubLoops->publish(msg);
                txtLog().info(THISMODULE "setLoops:%d",msg.data);
            }
        }

        void CSetLine::setSpd(float spd){
            if(m_pubSpd){
                std_msgs::msg::Float32 msg;
                msg.data = spd;
                m_pubSpd->publish(msg);
                txtLog().info(THISMODULE "setSpd:%f",msg.data);
            }
            
        }

        void CSetLine::setVehiTyp(uint8_t typ){
            if(0 == typ){
                RCLCPP_WARN(rclcpp::get_logger("setline"),"vehiTyp:%d wrong",typ);
                return;
            }
            if(m_pubVehiTyp){
                std_msgs::msg::UInt8 msg;
                msg.data = typ;
                m_pubVehiTyp->publish(msg);
                txtLog().info(THISMODULE "setVehiTyp:%d",msg.data);
            }
        }

        bool CSetLine::setExIds(const ids_t & exIds){
            ids_t tmpIds = m_s_ids;
            m_s_extIds.insert(exIds.begin(),exIds.end());
            for(const uint8_t & id:m_s_extIds){
                auto it = std::find(tmpIds.begin(),tmpIds.end(),id);
                if(it != tmpIds.end()){
                    tmpIds.erase(it);
                }               
            }
            //如确实有id从分组中排除则重新设置分组
            if(tmpIds.size() != m_s_ids.size()){
                setIds(tmpIds);
                geometry_msgs::msg::Polygon ofts;
                int idx=0;
                std::set<int>idxs;
                getExIdx<std::set<uint8_t> >(m_s_extIds,idxs);
                for(const geometry_msgs::msg::Point32 & pt:m_s_offsets.points){
                    //只添加没被排除的偏移
                    if(idxs.find(idx++) == idxs.end()){
                        ofts.points.push_back(pt);
                    }
                }
                setOffsets(ofts);

                if(isSwitchSetLine())
                    setWayPts(m_s_wayPts);
                if(isSwitchSetLoops())
                    setLoops(m_s_loops);
                return true;
            }else{
                //如从分组中未找到排除的id则不进行offset重新设置
                txtLog().info(THISMODULE "组中未找到排除的id则不进行重置");
                return false;
            }

        }

        void CSetLine::updateOffsets(const geometry_msgs::msg::Polygon &ofts){
            m_s_offsets = ofts;
            setOffsets(ofts);
        }

        void CSetLine::updateIds(const ids_t & ids){
            m_s_ids = ids;
            setIds(ids);
        }

        

        bool CSetLine::isInGroup(const ids_t & ids){
            auto it = std::find(ids.begin(),ids.end(),info::CParam::vehiId());
            return it != ids.end();
        }

        void CSetLine::setIds(const ids_t & ids){
            if(m_pubIds){
                std_msgs::msg::UInt8MultiArray msg;
                msg.data = ids;
                m_pubIds->publish(msg);
                txtLog().info(THISMODULE "设置同分组id:");
                int idx=-1;
                for(const uint8_t & id:ids)
                    txtLog().info(THISMODULE "ids[%d]=%d",++idx,id);
            }
        }

        void CSetLine::setOffsets(const geometry_msgs::msg::Polygon &ofts){
            
            if(m_pubOffsets){
                if(ofts.points.size() < 1){
                    RCLCPP_WARN(rclcpp::get_logger("setline"),"分组偏移为空");
                    return;
                }
                m_pubOffsets->publish(ofts);
                txtLog().info(THISMODULE "设置分组偏移:");
                int idx=-1;
                for(const geometry_msgs::msg::Point32 & pt:ofts.points)
                    txtLog().info(THISMODULE "offsets[%d] x:%f y:%f z:%f",
                    ++idx,pt.x,pt.y,pt.z);
            }
        }

        void CSetLine::updateParam(){
            //获取循环次数
            const Json::Value & lps = getParam(JsonParamIdx::LOOPS_IDX);
            m_s_loops = plugin::BasePlugin::getValue(lps);
            //获取载具类型
            const Json::Value & vehiTyp = getParam(JsonParamIdx::VEHICLE_TYP_IDX);
            if(!vehiTyp.isNull() && vehiTyp.isString()){
                if(vehiTyp.asString() == "多旋翼")
                    m_s_vehiType = Coper;
                else
                    m_s_vehiType = FixWing;
            }
            
            //获取速度
            m_s_spd = plugin::BasePlugin::getValue(getParam(JsonParamIdx::SPD_IDX));
            
            //获取到点距离
            m_s_arvDis = plugin::BasePlugin::getValue(getParam(JsonParamIdx::ARV_DIS_IDX));
            
            //获取航点信息,区域搜索时参数使用areaPoints，航线飞行时参数使用waypoints
            const Json::Value *pts = nullptr;
            if(info::CParam::actionName() == "SrchViaLine"){
                const Json::Value & pts2 = info::CParam::getParam("areaPoints");
                pts = &pts2;
            }else{
                const Json::Value & pts1 = getParam(JsonParamIdx::WAY_PTS_IDX);
                pts = &pts1;
            }
            
            if(!pts->isArray())
                return;
            m_s_wayPts.points.reserve(pts->size());
            m_s_wayPts.points.clear();
            for(const Json::Value & vv:*pts){
                if(!vv.isObject())
                    return;
                if(vv.isMember("x_lat") && vv.isMember("y_lon") && vv.isMember("z_alt")){
                    geometry_msgs::msg::Point32 pt;
                    geometry_msgs::msg::Point ptd;
                    pt.x=ptd.x = vv["x_lat"].asDouble();
                    pt.y=ptd.y = vv["y_lon"].asDouble();
                    pt.z=ptd.z = vv["z_alt"].asDouble();
                    algorithm::CUtility::checkZValid<float>(pt.z);
                    ptd.z=pt.z;
                    m_s_wayPts.points.push_back(pt);                
                }else{
                    return;
                }                   
            }
            
            //获取航点类型
            const Json::Value & ptTyp = getParam(JsonParamIdx::PT_TYP_IDX);
            int type = plugin::BasePlugin::Loc;
            if(!ptTyp.isNull() && ptTyp.isString()){
                type = ptTyp.asString()=="loc"?plugin::BasePlugin::Loc:plugin::BasePlugin::Gps;
            }
            if(plugin::BasePlugin::Gps == type){
                //gps转换为loc
                algorithm::CUtility::gps2loc(info::CParam::home(),m_s_wayPts.points);
            }
        }

        const Json::Value & CSetLine::getParam(JsonParamIdx idx){
            if(idx < 0 || idx > JsonParamIdx::PARAMS_COUNT)
                idx = (JsonParamIdx)0;
            std::string paramName = g_defParams[idx];
            getInput<std::string>(g_inputNames[idx],paramName);
            return info::CParam::getParam(paramName);
        }

    }
}