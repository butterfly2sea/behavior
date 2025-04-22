#ifndef ZYZN_MANAGE_ACTION_PLUGIN_MGR_HPP
#define ZYZN_MANAGE_ACTION_PLUGIN_MGR_HPP
#include <shared_mutex>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <plugin/base_plugin.hpp>

namespace zyzn{
    namespace manage{
        class ActionPluginMgr{
            public:
            /**
             * @对应指令等插件的支持情况
             */
            enum ProcType{
                UNSET = 0,   ///<! 未设置>
                SURPPORT,    ///<! 支持>    
                NOT_SURPPORT ///<! 不支持>
            };

            /**
             * @brief 行为控制类型
             */
            enum ActType{
                CTRL_VEHICLE = 0,  ///<! 控制飞机>
                CTRL_POD,          ///<! 控制吊舱>
                CTRL_COUNT
            };


            typedef std::shared_ptr<plugin::BasePlugin> PluginPtr_t;
            typedef std::pair<ProcType,PluginPtr_t>  ProcInfo_t;
            typedef std::pair<PluginPtr_t,ActType> ActInfo_t;//行为信息 first:处理行为插件 second:行为控制类型
            //命令处理插件映射，key为命令id，value为处理类型和插件指针，当类型未未设置时需要轮训全部插件以确定是否支持
            typedef std::map<int,ProcInfo_t >  CmdProc_t;

            ActionPluginMgr();

            virtual ~ActionPluginMgr();

            /**
             * @brief 处理json任务，依据任务名找出对应的处理插件，调用插件的参数处理函数后进行任务执行
             * @param action json任务信息 包括各参数
             * @param cmd 任务命令类型 如："start","pause","continue","stop","set"
             * @param sn 任务序列号
             */
            virtual void procJsonAction(const Json::Value & action,const char* cmd,int sn);

            /**
             * @brief 处理收到的命令信息，找到能响应该指令的插件进行处理，依据获取的行为树名进行处理
             * @param msg 命令信息
             * @param result out 插件返回的行为树名
             * @return true 处理成功 false 处理失败
             */
            virtual bool procCmd(const custom_msgs::msg::CommandRequest::SharedPtr msg,std::string & result);

            /**
             * @brief 处理摇杆控制信息，找到对应的处理插件,首次调用时需要轮训全部插件以确定是否支持
             * @param result out 插件返回的行为树名
             * @return true 处理成功 false 处理失败
             */
            virtual bool procJoy(std::string & result);

            /**
             * @brief 处理跟踪及打击信息，找到对应的处理插件,首次调用时需要轮训全部插件以确定是否支持
             * @param msg 跟踪及打击目标信息
             * @param result out 插件返回的行为树名
             * @return true 处理成功 false 处理失败
             */
            virtual bool procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,std::string & result);

            /**
             * @brief 加载插件，加载后创建插件对象，并
             * @param path 插件路径
             * @param name 插件配置文件名
             * @return true 加载成功 false 加载失败
             */
            virtual bool loadPlugin(const std::string & path,const std::string & name);

            protected:

            /**
             * @brief 行为树线程运行函数,检查任务状态，成功完成后进入悬停状态，失败时隔15秒重新执行行为，
             * 运行状态航点变化时才上报状态
             */
            virtual void treeRun();

            private:
            /**
             * @brief 更新当前执行行为树
             * @param treeName in 行为树名
             * @param result out 执行失败时 存放失败相关提示信息
             * @return true:更新成功 false:更新失败
             */
            bool updateTree(const std::string & treeName,std::string & result,ActType typ = ActType::CTRL_VEHICLE);

            /**
             * @brief 初始化json任务信息，重置任务状态及航点信息
             * @param sn in 任务序号
             */
            void initJsonAction(int sn);

            /**
             * @brief 保存任务中的参数及触发信息
             * @param params in 参数信息
             * @param triggers in 触发信息
             */
            void saveParamAndTrigger(const Json::Value & params,const Json::Value & triggers);

            /**
             * @brief 依据行为信息获取行为名和行为控制类型
             * @param actInfo in 行为信息 格式为行为名-控制类型,当控制类型为飞机控制时可忽略-控制类型
             * @param actName out 从actInfo中获取的行为名
             * @param actTyp  out 从actInfo中获取控制类型
             * 
             */
            void getActInfo(std::string actInfo,std::string & actName,ActType & actTyp);

            /**
             * @brief 判断是否有有效树指针
             */
            inline bool hasValidTree(){
                for(int i=0;i<ActType::CTRL_COUNT;++i){
                    if(m_curRunningTree[i] && m_curRunningTree[i].get())
                        return true;
                }
                return false;
            }
            protected:
            std::shared_ptr<std::thread> m_trdTree;     //行为树执行线程
            std::shared_ptr<BT::Tree> m_curRunningTree[ActType::CTRL_COUNT]; //当前运行树
            BT::BehaviorTreeFactory m_factory;          //树工厂对象           
            bool    m_isRunning;                        //树是否运行
            bool    m_ctrlViaJson;
            std::shared_timed_mutex m_mtx;              //树互斥对象

            CmdProc_t m_cmdProc;                          //命令处理插件映射
            ProcInfo_t m_traceAttackProc;                 //跟踪攻击行为处理插件
            ProcInfo_t m_joyProc;                         //摇杆行为处理插件
            std::map<std::string,ActInfo_t>  m_actProc; //行为处理插件映射，key为行为名，value为行为处理信息(插件及控制类型)
            std::vector<PluginPtr_t> m_plugins;            //插件列表

        };
    }
}

#endif