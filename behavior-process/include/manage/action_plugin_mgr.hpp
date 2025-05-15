#ifndef ZYZN_MANAGE_ACTION_PLUGIN_MGR_HPP
#define ZYZN_MANAGE_ACTION_PLUGIN_MGR_HPP
#include <shared_mutex>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <action_libs/base_plugin.hpp>

namespace zyzn{
    namespace manage{
        class ActionPluginMgr{
            public:

            enum ProcType{
                UNSET = 0,   ///<! 未设置>
                SURPPORT,    ///<! 支持>    
                NOT_SURPPORT ///<! 不支持>
            };


            typedef std::shared_ptr<plugin::BasePlugin> PluginPtr_t;
            typedef std::pair<ProcType,PluginPtr_t>  ProcInfo_t;
            //命令处理插件映射，key为命令id，value为处理类型和插件指针，当类型未未设置时需要轮训全部插件以确定是否支持
            typedef std::map<int,ProcInfo_t >  CmdProc_t;

            ActionPluginMgr();

            ~ActionPluginMgr();

            void procJsonAction(const Json::Value & action,const char* cmd,int sn);

            bool procCmd(const custom_msgs::msg::CommandRequest::SharedPtr msg,std::string & result);

            bool procJoy(std::string & result);

            bool procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,std::string & result);

            bool loadPlugin(const std::string & path,const std::string & name);

            private:

            void treeRun();

            bool updateTree(const std::string & treeName,std::string & result);

            void sendSts(int sts,const char* info="");

            void initJsonAction(int sn);
            private:
            std::shared_ptr<std::thread> m_trdTree;     //行为树执行线程
            std::shared_ptr<BT::Tree> m_curRunningTree; //当前运行树
            BT::BehaviorTreeFactory m_factory;          //树工厂对象           
            bool    m_isRunning;                        //树是否运行
            bool    m_ctrlViaJson;
            std::shared_timed_mutex m_mtx;              //树互斥对象

            CmdProc_t m_cmdProc;                          //命令处理插件映射
            ProcInfo_t m_traceAttackProc;                 //跟踪攻击行为处理插件
            ProcInfo_t m_joyProc;                         //摇杆行为处理插件
            std::map<std::string,PluginPtr_t>  m_actProc; //行为处理插件映射，key为行为名，value为插件指针
            std::vector<PluginPtr_t> m_plugins;            //插件列表
        };
    }
}

#endif