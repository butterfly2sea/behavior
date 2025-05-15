#include "../../include/check/CheckArriveDst.hpp"
#include "../../include/utility/Utility.hpp"
#include "../../include/get/GetLocation.hpp"
#include "../../include/info/OffboardInfo.hpp"
namespace zyzn{
    namespace check{
        CCheckArriveDst::CCheckArriveDst(const std::string& name,
        const NodeConfig& conf):SyncActionNode(name,conf){
            
        }
        
        PortsList CCheckArriveDst::providedPorts(){
            return {InputPort<custom_msgs::msg::OffboardCtrl>("target"),
            InputPort<float>("arvdis"),
            InputPort<int>("onlyz")
            };
        }

        NodeStatus CCheckArriveDst::tick(){

            float arvDis = 0.2;
            int onlyZ = 1;
            //到点判定距离
            Expected<float> dis = getInput<float>("arvdis");
            if(dis)
                arvDis = dis.value();
            
            //是否只判定垂直向是否到达
            Expected<int> z = getInput<int>("onlyz");
            if(z)
                onlyZ = z.value();
            
            //本机目标点位置
            custom_msgs::msg::OffboardCtrl dst;
            getInput<custom_msgs::msg::OffboardCtrl>("target",dst);
            
            float ds = algorithm::CUtility::getDisFrmLoc(dst.x,dst.y,dst.z,
            get::CGetlocation::simpVehi().x/1e3,get::CGetlocation::simpVehi().y/1e3,
            get::CGetlocation::simpVehi().z/1e3);
            //判定是否到点，当只需判定本机是否到点时，依据其距离目的点距离进行判定，当同时需要判定其它飞机时，
            //本机及他机同时到点5s后认为到点
            if(ds <= arvDis){                        
                RCLCPP_INFO(rclcpp::get_logger("CheckArriveDst"),"arrive dst self");
                return NodeStatus::SUCCESS;                           
            }
            
            return NodeStatus::FAILURE;
            
        }

    }
}