#include <json/json.h>
#include <log/Logger.hpp>
#include "../../include/status/Weapon.hpp"
#include "../../include/info/Param.hpp"

namespace zyzn{
    namespace status{

        Weapon::Weapon():m_subSts(nullptr),
        m_pubCtrl(nullptr),m_sts(-1){
            m_pubCtrl  = info::CParam::rosNode()->create_publisher<std_msgs::msg::UInt8>
            ("inner/control/weapon",10);
            m_subSts = info::CParam::rosNode()->create_subscription<std_msgs::msg::UInt8>(
            "inner/information/gpio_status",
            rclcpp::SensorDataQoS(),std::bind(&Weapon::statusCallback,this,std::placeholders::_1));
        }

        void Weapon::weaponCtrl(int enable,int cmd){
            std_msgs::msg::UInt8 weapon;
            weapon.data = WEAPON_FUN_NUM*enable+cmd;
            m_pubCtrl->publish(weapon);
            txtLog().info(THISMODULE "ctrl weapon enable:%d cmd:%d msgdata:%d",enable,cmd,weapon.data);
        }

        void Weapon::respSts(){
            
            m_cmdResp.cmdRspMsg().src = 0;//指令类型
            m_cmdResp.cmdRspMsg().type = CMD_TYPE;//设备信息
            Json::Value devs(Json::arrayValue);
            Json::Value weaponSts(Json::objectValue);
            weaponSts["type"]=DEV_TYPE;//weapon
            weaponSts["id"] = 1;
            weaponSts["status"] = UNKNOWN;//设备状态
            if(m_sts>=0){
                weaponSts["status"] = OPEN;
                Json::Value customs(Json::objectValue);
                customs["unlocker"] = m_sts & UNLOCKER;
                customs["trigger"] = (m_sts & TRIGGER)>>1;
                weaponSts["custom"] = customs;
            }
            devs.append(weaponSts);
            m_cmdResp.cmdRspMsg().rslt = devs.toStyledString();
            m_cmdResp.sendCmdRsp();
            //RCLCPP_INFO(rclcpp::get_logger("weaponCtrl"),"weapon sts:%s",cmdResp.rslt.c_str());
        }

        void Weapon::statusCallback(const std_msgs::msg::UInt8::SharedPtr sts){
            if(m_sts != sts->data){
                m_sts = sts->data;
                respSts();
            }

        }

    }
}