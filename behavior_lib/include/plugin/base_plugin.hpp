#ifndef ZYZN_PLUGIN_BASE_PLUGIN_HPP
#define ZYZN_PLUGIN_BASE_PLUGIN_HPP
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <boost/config.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <json/json.h>
#include <custom_msgs/msg/command_request.hpp>
#include <custom_msgs/msg/object_attack_designate.hpp>

namespace zyzn{
    namespace plugin{
        /**
         * @brief 插件基类，所有插件必须继承该类，每个插件支持的行为(可为多个)和地面站的一个指令(json任务)对应
         * 同时为了响应定制其它指令，定义了命令处理、遥杆处理及其它，每个指令只被一个插件处理
         */
        class BOOST_SYMBOL_VISIBLE  BasePlugin{
            public:
            typedef std_msgs::msg::UInt8MultiArray::_data_type ids_t;
            /**
             * 航点类型
            */
            enum EPointType{
                Loc=0,///<!loc位置>
                Gps   ///<!gps位置>
            };

            BasePlugin(){

            }
            virtual ~BasePlugin(){

            }

            /**
             * @brief 从json中解析任务参数
             * @param in json串
             * @return 是否解析成功，参数是否有效
             */
            virtual bool parseJsonParam(const Json::Value &params){
                return true;
            };

            /**
             * @brief 注册行为树节点,注册插件中所有行为树节点
             * @param factory 行为树工厂
             */
            virtual void registerNode(BT::BehaviorTreeFactory &factory){
                
            }
            
            /**
             * 
             * @brief 处理命令指令
             * @param in msg 指令
             * @param out treeName 需要用行为树执行时，行为树名称
             * @return 是否处理成功，如果返回true，则后续插件不再处理该指令
             */
            virtual bool procCmd(const custom_msgs::msg::CommandRequest::SharedPtr msg,std::string & treeName){
                return false;
            }

            /**
             * @brief 处理遥杆指令
             * @param out treeName 需要用行为树执行时，行为树名称
             * @return 是否处理成功，如果返回true，则后续插件不再处理该指令
             */
            virtual bool procJoyCtrl(std::string & treeName){
                return false;
            }

            /**
             * @brief 处理跟踪攻击指令
             * @param in msg 指定跟踪打击目标信息
             * @param out treeName 需要用行为树执行时，行为树名称
             * @return 是否处理成功，如果返回true，则后续插件不再处理该指令
             */
            virtual bool procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,std::string & treeName){
                return false;
            }

        
            /**
             * @brief 将json值转换为数字并返回
             * @param in info 任务信息
             * @result 数字值
            */
            static inline double getValue(const Json::Value & info){
                if(info.isNumeric()){
                    return info.asDouble();
                }else if(info.isString()){
                    return atof(info.asCString());
                }else if(info.isObject() && info.isMember("value")){
                    if(info["value"].isNumeric()){
                        return info["value"].asDouble();
                    }else if(info["value"].isString()){
                        return atof(info["value"].asCString());
                    }
                }
                return 0;
            }

            private:
            

        };

        


    }
}

#endif

