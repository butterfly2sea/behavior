#ifndef ZYZN_INFO_PARAM_HPP
#define ZYZN_INFO_PARAM_HPP
#include <sys/types.h>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <json/json.h>
#include <geometry_msgs/msg/point.hpp>
namespace zyzn{
    namespace info{

        /**
         * @brief 参数信息保存 主要用来进行任务中相关参数保存，各结点均可访问
         * @author zyzn
        */
        class CParam{
            public:

            inline static u_int8_t & vehiId(){
                return m_s_selfId;
            }

            inline static geometry_msgs::msg::Point & home(){
                return m_s_home;
            }

            inline static  rclcpp::Node::SharedPtr & rosNode(){
                return m_s_glbNode;
            }

            inline static void updateParam(const std::string & name,const Json::Value & value){
                m_s_jsonParams[name] = value;
            }

            inline static const Json::Value & getParam(const std::string & name){
                return m_s_jsonParams[name];
            }

            inline static void updateTrigger(const char* name,const Json::Value & value){
                m_s_jsonTriggers[name] = value;
            }

            inline static const Json::Value & getTrigger(const std::string & name){
                return m_s_jsonTriggers[name];
            }

            inline static std::string & actionName(){
                return m_s_actionName;
            }

            private:

            static u_int8_t m_s_selfId;//本机id
            
            static geometry_msgs::msg::Point m_s_home;//设置的home点
            
            static rclcpp::Node::SharedPtr m_s_glbNode;//全局rosnode用于行为节点非tick时也能接收消息
            
            static std::map<std::string,Json::Value> m_s_jsonParams;//json任务中的参数信息<参数名，参数值>

            static std::map<std::string,Json::Value> m_s_jsonTriggers;//json任务中的触发信息

            static std::string m_s_actionName;//当前执行的任务名称

        };
    }
}

#endif