#include "get/GetTraceInfo.hpp"
#include <behavior_lib/set/SetLine.hpp>
#include <behavior_lib/info/Param.hpp>

namespace zyzn{
    namespace get{

            CTraceInfo::CTraceInfo(const std::string &name,const BT::NodeConfig& config):SyncActionNode(name,config){

            }
            PortsList CTraceInfo::providedPorts(){
                return { OutputPort<float>("angle"),//相差角度
                OutputPort<float>("tracez"),//跟踪高度
                OutputPort<int>("followid"),//跟随飞机id和此飞机保持 相差角度（可能不用）
                OutputPort<int>("layidx"),  //处于分层中的索引
                InputPort<int>("ztyp"),     //高度确定方式 0：使用当前高度，1：使用分层高度，默认使用当前高度方式
                InputPort<float>("zintval")};//垂向间距
        
            }

            NodeStatus CTraceInfo::tick() {
                if(!CGetGroupLocation::hasAllVehis())
                    return NodeStatus::FAILURE;
                compTraceInfo();
                return NodeStatus::SUCCESS;
            }
            CGetGroupLocation::id_t CTraceInfo::getNearVehi(float z,const CGetGroupLocation::vehis_t &vehis){
                float minZ = CGetGroupLocation::Highest;
                id_t id = CGetGroupLocation::InvalId;
                for(auto it=vehis.begin();it!=vehis.end();++it){
                    if(abs(it->second.z/1e3 - z) < abs(minZ-z) && m_exId.find(it->first)==m_exId.end()){
                        id = it->first;
                        minZ = it->second.z/1e3;
                    }
                }
                return id;
            }

            void CTraceInfo::compTraceInfo(){
                int ztyp = SelfZ;
                float z = 0,zIntval = 20;
                CGetGroupLocation::id_t id = CGetGroupLocation::InvalId;
                getInput<int>("ztyp",ztyp);
                if(LayerZ == ztyp){
                    getInput<float>("zintval",zIntval);
                    CGetGroupLocation::getMinZ(id,z);
                }else
                    zIntval = 0;
                if(zIntval < 0)
                    zIntval *= -1;
                const CGetGroupLocation::vehis_t &vehis = CGetGroupLocation::allVehis();
                m_layer.clear();
                m_exId.clear();
                m_layer.reserve(set::CSetLine::ids().size());
            
                while(m_layer.size() < set::CSetLine::ids().size()){
                    id = getNearVehi(z,vehis);
                    if(CGetGroupLocation::InvalId != id){
                        m_layer.push_back(id);
                        m_exId.insert(id);
                        if(info::CParam::vehiId() == id)
                            break;
                    }
                    z -= zIntval;
                }
                setOutput<int>("layidx",m_layer.size()-1);
                id = CGetGroupLocation::InvalId;
                if(m_layer.size()>1){
                    id = m_layer[m_layer.size()-2];
                }
                setOutput<int>("followid",id);
                if(SelfZ == ztyp){
                    geometry_msgs::msg::Point32 pt;
                    CGetGroupLocation::getSelfLoc(pt);
                    z = pt.z;
                }
                setOutput<float>("tracez",z);
                float angle = algorithm::CUtility::ang2rad(360.0/set::CSetLine::ids().size());
                setOutput<float>("angle",angle);
            }
    }
}