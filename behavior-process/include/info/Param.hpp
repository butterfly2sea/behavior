#ifndef ZYZN_INFO_PARAM_HPP
#define ZYZN_INFO_PARAM_HPP
#include <sys/types.h>
#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/object_location.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <custom_msgs/msg/dis_target.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace zyzn{
    namespace info{
        
        /**
         * 航点类型
        */
        enum EPointType{
            LocPt=0, ///<!loc航点>
            GpsPt    ///<!gps航点>
        };

        /**
         * @brief 参数信息保存
         * @details 主要用来进行任务中相关参数保存，各结点均可访问
         * @author zyzn
        */
        class CParam{
            public:

            static u_int8_t m_sSelfId;//本机id
            
            static custom_msgs::msg::TaskStage m_sCurStage;//当前阶段任务
            static geometry_msgs::msg::Point m_sHome;//设置的home点
            static custom_msgs::msg::DisTarget m_sCurNavInfo;
            static int m_sPreNavId;
            static rclcpp::Node::SharedPtr m_glbNode;//全局rosnode用于行为节点非tick时也能接收消息

        };
    }
}

#endif