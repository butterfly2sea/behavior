#include "../include/info/Param.hpp"
#include "../include/utility/Utility.hpp"

namespace zyzn{
    namespace info{
        u_int8_t CParam::m_sSelfId = 0;

        custom_msgs::msg::TaskStage CParam::m_sCurStage;

        custom_msgs::msg::DisTarget CParam::m_sCurNavInfo;
        int CParam::m_sPreNavId=-1;
        geometry_msgs::msg::Point CParam::m_sHome;

        rclcpp::Node::SharedPtr CParam::m_glbNode=nullptr;

    }
    
}