#include "../include/info/Param.hpp"
#include "../include/utility/Utility.hpp"

namespace zyzn{
    namespace info{
        u_int8_t CParam::m_s_selfId = 0;
        geometry_msgs::msg::Point CParam::m_s_home;
        rclcpp::Node::SharedPtr CParam::m_s_glbNode=nullptr;
        std::map<std::string,Json::Value> CParam::m_s_jsonParams;
        std::map<std::string,Json::Value> CParam::m_s_jsonTriggers;
        std::string CParam::m_s_actionName;
    }
    
}