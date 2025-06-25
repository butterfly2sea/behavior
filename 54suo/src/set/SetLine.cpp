#include <rclcpp/rclcpp.hpp>
#include "../../include/set/SetLine.hpp"
#include "../../include/info/Param.hpp"

namespace zyzn{
    namespace set{

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
            init();
        }
        CSetLine::CSetLine():SyncActionNode("",BT::NodeConfig()){
            init();
        }


        PortsList CSetLine::providedPorts(){
            return {
            InputPort<float>("antiDis"), //避撞距离
            InputPort<int>("type"),      //设置内容类型
            OutputPort<uint32_t>("delay")//延迟时间，编队飞行延迟时间
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
            uint32_t delay = info::CParam::m_sCurStage.trigger.param1<0?0:(info::CParam::m_sCurStage.trigger.param1*1000);
            setOutput<uint32_t>("delay",delay);
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
            m_setTyp = SetContentTyp::ALL;
            m_pubTime = info::CParam::m_glbNode->create_publisher<std_msgs::msg::Int64>(
                "inner/set/form_time",1);
            if(WAY_PTS&m_setTyp)
                m_pubLine = info::CParam::m_glbNode->create_publisher<geometry_msgs::msg::Polygon>(
                "inner/set/navline",1);
            if(ANTI_DIS&m_setTyp)
                m_pubCollDis = info::CParam::m_glbNode->create_publisher<std_msgs::msg::Float32>(
                "inner/set/dis_anti_collide",1);
            if(ARV_DIS&m_setTyp)
                m_pubArvDis = info::CParam::m_glbNode->create_publisher<std_msgs::msg::Float32>(
                "inner/set/dis_arrive",1);
            if(FORM&m_setTyp)
                m_pubForm = info::CParam::m_glbNode->create_publisher<custom_msgs::msg::ParamShort>(
                "inner/set/form",1);
            if(GROUP&m_setTyp)
                m_pubGrp = info::CParam::m_glbNode->create_publisher<std_msgs::msg::UInt8>(
                "inner/set/group",1);
            if(LOOPS&m_setTyp)
                m_pubLoops = info::CParam::m_glbNode->create_publisher<std_msgs::msg::Int32>(
                "inner/set/line_loops",1);
            if(SPD&m_setTyp)
                m_pubSpd = info::CParam::m_glbNode->create_publisher<std_msgs::msg::Float32>(
                "inner/set/form_spd",1);
            if(VEHICLE_TYP&m_setTyp)
                m_pubVehiTyp = info::CParam::m_glbNode->create_publisher<std_msgs::msg::UInt8>(
                "inner/set/vehicle_type",1);
            if(IDS&m_setTyp)
                m_pubIds = info::CParam::m_glbNode->create_publisher<std_msgs::msg::UInt8MultiArray>
            ("inner/information/group_ids",1);
            if(OFFSETS&m_setTyp)
                m_pubOffsets = info::CParam::m_glbNode->create_publisher<geometry_msgs::msg::Polygon>
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
                RCLCPP_INFO(rclcpp::get_logger("setline"),"setArvDis:%f",msg.data);
            }           
        }

        void CSetLine::setForm(const custom_msgs::msg::ParamShort& typ){
            if(m_pubForm){
                m_pubForm->publish(typ);
                RCLCPP_INFO(rclcpp::get_logger("setline"),"setForm:%d",typ.type);
            }
            
        }

        void CSetLine::setGrp(uint8_t grp){
            if(m_pubGrp){
                std_msgs::msg::UInt8 msg;
                msg.data = grp;
                m_pubGrp->publish(msg);
                RCLCPP_INFO(rclcpp::get_logger("setline"),"setGrp:%d",msg.data);
            }
            
        }

        void CSetLine::setWayPts(const geometry_msgs::msg::Polygon& pts){
            if(m_pubLine){
                int64_t nt = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
                //当前时间+延迟时间 为航线飞行开始时间
                nt += info::CParam::m_sCurStage.trigger.param1<0?0:(info::CParam::m_sCurStage.trigger.param1*1000);
                std_msgs::msg::Int64 lineSt;
                lineSt.data = nt;
                m_pubTime->publish(lineSt);//set start time
                m_pubLine->publish(pts);
                RCLCPP_INFO(rclcpp::get_logger("setline"),"setline pts number:%d",pts.points.size());
            }
        }

        void CSetLine::setLoops(int lp){
            if(m_pubLoops){
                std_msgs::msg::Int32 msg;
                msg.data = lp;
                m_pubLoops->publish(msg);
                RCLCPP_INFO(rclcpp::get_logger("setline"),"setLoops:%d",msg.data);
            }
        }

        void CSetLine::setSpd(float spd){
            if(m_pubSpd){
                std_msgs::msg::Float32 msg;
                msg.data = spd;
                m_pubSpd->publish(msg);
                RCLCPP_INFO(rclcpp::get_logger("setline"),"setSpd:%f",msg.data);
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
                RCLCPP_INFO(rclcpp::get_logger("setline"),"setVehiTyp:%d",msg.data);
            }
        }

        bool CSetLine::setExIds(const ids_t & exIds){
            ids_t tmpIds = info::CParam::m_sCurStage.head.ids;
            m_s_extIds.insert(exIds.begin(),exIds.end());
            for(const uint8_t & id:m_s_extIds){
                auto it = std::find(tmpIds.begin(),tmpIds.end(),id);
                if(it != tmpIds.end()){
                    tmpIds.erase(it);
                }               
            }
            //如确实有id从分组中排除则重新设置分组
            if(tmpIds.size() != info::CParam::m_sCurStage.head.ids.size()){
                setIds(tmpIds);
                geometry_msgs::msg::Polygon ofts;
                int idx=0;
                std::set<int>idxs;
                getExIdx<std::set<uint8_t> >(m_s_extIds,idxs);
                for(const geometry_msgs::msg::Point32 & pt:info::CParam::m_sCurStage.formoffset.points){
                    //只添加没被排除的偏移
                    if(idxs.find(idx++) == idxs.end()){
                        ofts.points.push_back(pt);
                    }
                }
                setOffsets(ofts);
                if(isSwitchSetLine())
                    setWayPts(info::CParam::m_sCurStage.line);
                if(isSwitchSetLoops())
                    setLoops(m_s_loops);
                return true;
            }else{
                //如从分组中未找到排除的id则不进行offset重新设置
                RCLCPP_INFO(rclcpp::get_logger("setline"),"组中未找到排除的id则不进行重置");
                return false;
            }

        }

        void CSetLine::updateOffsets(const geometry_msgs::msg::Polygon &ofts){
            info::CParam::m_sCurStage.formoffset = ofts;
            setOffsets(ofts);
        }

        void CSetLine::updateIds(const ids_t & ids){
            info::CParam::m_sCurStage.head.ids = ids;
            setIds(ids);
        }

        

        bool CSetLine::isInGroup(const ids_t & ids){
            auto it = std::find(ids.begin(),ids.end(),info::CParam::m_sSelfId);
            return it != ids.end();
        }

        void CSetLine::setIds(const ids_t & ids){
            if(m_pubIds){
                std_msgs::msg::UInt8MultiArray msg;
                msg.data = ids;
                m_pubIds->publish(msg);
                RCLCPP_INFO(rclcpp::get_logger("setline"),"设置同分组id:");
                int idx=-1;
                for(const uint8_t & id:ids)
                    RCLCPP_INFO(rclcpp::get_logger("setline"),"ids[%d]=%d",++idx,id);
            }
        }

        void CSetLine::setOffsets(const geometry_msgs::msg::Polygon &ofts){
            
            if(m_pubOffsets){
                if(ofts.points.size() < 1){
                    RCLCPP_WARN(rclcpp::get_logger("setline"),"分组偏移为空");
                    return;
                }
                m_pubOffsets->publish(ofts);
                RCLCPP_INFO(rclcpp::get_logger("setline"),"设置分组偏移:");
                int idx=-1;
                for(const geometry_msgs::msg::Point32 & pt:ofts.points)
                    RCLCPP_INFO(rclcpp::get_logger("setline"),"offsets[%d] x:%f y:%f z:%f",
                    ++idx,pt.x,pt.y,pt.z);
            }
        }

    }
}