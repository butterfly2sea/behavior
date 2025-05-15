#ifndef ZYZN_MANAGE_ACTIONS_HPP
#define ZYZN_MANAGE_ACTIONS_HPP

#include <vector>
#include <list>
#include <map>
#include <boost/signals2/signal.hpp>
#include <json/json.h>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <std_msgs/msg/string.hpp>

#include "../set/SetLine.hpp"

namespace zyzn{
    namespace manage{
        /**
         * @brief json任务解析
         * @details 依据收到的json任务，从中解析出任务参数，并个触发对应的执行信号
         * @author zyzn
        */
        class CActionsMgr{

        public:      

            typedef void (f_dotask_t) ( const Json::Value &,const char*,int);
            typedef bool (f_setExtIds_t)(const set::CSetLine::ids_t &);
            typedef boost::signals2::signal<f_dotask_t> sig_procJsonAct_t;
            typedef boost::signals2::signal<f_setExtIds_t> sig_setExtIds_t;

            typedef std::map<int,Json::Value> mission_t;
            typedef std::map<int,Json::Value>::iterator mission_it_t;
            
            sig_setExtIds_t sig_setExtIds;
            sig_procJsonAct_t sig_procJsonAct;

            CActionsMgr();
            ~CActionsMgr();
            /**
             * @brief 处理阶段任务,处理json中的任务信息，包括获取组id、组成员id、参数信息、控制指令
             * @param json_str in 任务信息
            */
            void processStage(const Json::Value &json_str);

            /**
             * @brief 收到的json任务回调函数,获取到任务信息后进行json任务处理
             * @param in msg json任务消息内容
            */
            void jsonTaskCallback(const std_msgs::msg::String::SharedPtr msg);
            private:

            bool getFormoffset(int grpId,const Json::Value &actions,std::vector<geometry_msgs::msg::Point32> & rslt);

            
        };
        
        
    }
}



#endif