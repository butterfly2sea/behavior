#include <fstream>
#include <boost/dll/import.hpp>
#include <manage/action_plugin_mgr.hpp>
#include <status/CommandStatus.hpp>
#include <set/SetLine.hpp>
#include <action_libs/core_plugin.hpp>
namespace dll = boost::dll;
namespace zyzn {
    namespace manage {
        static std::vector<dll::shared_library> g_s_libs;
        ActionPluginMgr::ActionPluginMgr() {
            m_traceAttackProc.first = UNSET;
            m_joyProc.first = UNSET;
            m_isRunning = true;
            m_trdTree = std::make_shared<std::thread>(std::bind(&ActionPluginMgr::treeRun,this));
        }

        ActionPluginMgr::~ActionPluginMgr() {
            m_isRunning = false;
            m_trdTree->join();
        }

        void ActionPluginMgr::procJsonAction(const Json::Value & action,const char* cmd,int sn){
            initJsonAction(sn);
            if(!action.isMember("name")){
                status::CCommandStatus::sendTaskStatus();
                return;
            }
            std::string actName = action["name"].asString();
            if(m_actProc.find(actName) == m_actProc.end() || !m_actProc[actName]){
                status::CCommandStatus::sendTaskStatus();
                return;
            }
            if(!action.isMember("params") || !action["params"].isArray()){
                status::CCommandStatus::sendTaskStatus();
                return;
            }
            const Json::Value & params = action["params"];
            if(!m_actProc[actName]->parseJsonParam(params)){
                status::CCommandStatus::sendTaskStatus();
                return;
            }
            std::string treeName(action["name"].asString());
            treeName += "-";
            treeName += cmd;
            if(updateTree(treeName,status::CCommandStatus::cmdRspMsg().rslt))
                status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsNoStart;
            status::CCommandStatus::sendTaskStatus();
        }

        bool ActionPluginMgr::procCmd(const custom_msgs::msg::CommandRequest::SharedPtr msg,std::string & result){
            m_ctrlViaJson = false;
            std::string treeName;
            if(m_cmdProc.find(msg->type) == m_cmdProc.end()){
                m_cmdProc[msg->type].first = NOT_SURPPORT;
                result = "无插件支持此命令";
                for(auto & plugin : m_plugins){
                    if(plugin->procCmd(msg,treeName)){
                        m_cmdProc[msg->type].first = SURPPORT;
                        m_cmdProc[msg->type].second = plugin;
                        result = "";
                        break;
                    }
                }
            }else if(SURPPORT == m_cmdProc[msg->type].first){
                m_cmdProc[msg->type].second->procCmd(msg,treeName);
            }else if(NOT_SURPPORT == m_cmdProc[msg->type].first){
                result = "无插件支持此命令";
                return false;
            }
            return updateTree(treeName,result);
        }

        bool ActionPluginMgr::procJoy(std::string & result){
            m_ctrlViaJson = false;
            std::string treeName;
            if(SURPPORT == m_joyProc.first){
                m_joyProc.second->procJoyCtrl(treeName);
            }else if(NOT_SURPPORT == m_joyProc.first){
                result = "无插件支持摇杆控制行为";
                return false;
            }else if(UNSET == m_joyProc.first){
                for(auto & plugin : m_plugins){
                    if(plugin->procJoyCtrl(treeName)){
                        m_joyProc.first = SURPPORT;
                        m_joyProc.second = plugin;
                        break;
                    }
                }
                if(UNSET == m_joyProc.first){
                    m_joyProc.first = NOT_SURPPORT;
                    result = "无插件支持摇杆控制行为";
                    return false;
                }
            }
            
            return updateTree(treeName,result);
        }

        bool ActionPluginMgr::procTraceAttack(const custom_msgs::msg::ObjectAttackDesignate::ConstSharedPtr msg,
        std::string & result){
            m_ctrlViaJson = false;
            std::string treeName;
            if(SURPPORT == m_traceAttackProc.first){
                m_traceAttackProc.second->procTraceAttack(msg,treeName);
            }else if(NOT_SURPPORT == m_traceAttackProc.first){
                result = "无插件支持跟踪打击行为";
                return false;
            }else if(UNSET == m_traceAttackProc.first){
                for(auto & plugin : m_plugins){
                    if(plugin->procTraceAttack(msg,treeName)){
                        m_traceAttackProc.first = SURPPORT;
                        m_traceAttackProc.second = plugin;
                        break;
                    }
                }
                if(UNSET == m_traceAttackProc.first){
                    m_traceAttackProc.first = NOT_SURPPORT;
                    result = "无插件支持跟踪打击行为";
                    return false;
                }
            }
            return updateTree(treeName,result);
        }

        bool ActionPluginMgr::loadPlugin(const std::string & path,const std::string & name) {
            
            std::ifstream ifs;
            bool addDirSep = path.find_last_of("/")!=path.length()-1;
            std::string treePath=(addDirSep?"/":"");
            treePath += "tree/";
            std::string configPath = path + treePath + name;
            ifs.open(configPath,std::ifstream::in);
            if(!ifs.is_open()){
                return false;
            }
            Json::Value root;
            Json::CharReaderBuilder builder;
            Json::String err;
            if(!Json::parseFromStream(builder,ifs,&root,&err)){
                return false;
            }
            if(root["plugins"].isNull() || root["plugins"].isArray() == false){
                return false;
            }
            if(root["xmlFiles"].isNull() || root["xmlFiles"].isArray() == false){
                return false;
            }
            m_plugins.reserve(10);
            g_s_libs.reserve(10);
            //创建coreplugin
            std::shared_ptr<plugin::BasePlugin> corePlugin = std::make_shared<plugin::CorePlugin>();
            corePlugin->registerNode(m_factory);
            m_plugins.push_back(corePlugin);
            if(root.isMember("coreActions") && root["coreActions"].isArray()){
                const Json::Value & coreActions = root["coreActions"];
                for(const Json::Value & coreAct : coreActions)
                    m_actProc[coreAct.asString()] = corePlugin;
            }

            Json::Value & plugins = root["plugins"];
            Json::Value & xmlFiles = root["xmlFiles"];
            for(const Json::Value & plugin : plugins){
                if(!plugin.isMember("name"))
                    continue;
                std::string pluginName = path + treePath + plugin["name"].asString();
                dll::shared_library lib(pluginName, dll::load_mode::append_decorations);
                
                if (!lib.has("create_plugin")) 
                    continue;
                g_s_libs.push_back(lib);
                using plugin_create_t = PluginPtr_t ();
                auto creator = dll::import_alias<plugin_create_t>(
                    g_s_libs[g_s_libs.size()-1], "create_plugin"
                );
                PluginPtr_t plg = creator();
                plg->registerNode(m_factory);
                m_plugins.push_back(plg);
                if(!plugin.isMember("actions") || !plugin["actions"].isArray())
                    continue;
                const Json::Value & acts = plugin["actions"];
                for(const Json::Value & act:acts)
                    m_actProc[act.asString()] = plg;
            }
            for(const Json::Value & xmlFile:xmlFiles){
                std::string xmlName = path + treePath + xmlFile.asString();
                m_factory.registerBehaviorTreeFromFile(xmlName);
            }
            ifs.close();
            return true;
        }

        void ActionPluginMgr::treeRun(){
            time_t preTm = time(nullptr);
            time_t tn;
            
            while(m_isRunning ){
                NodeStatus sts = NodeStatus::RUNNING;
                bool complete = false;
                {
                    try{
                    std::shared_lock<std::shared_timed_mutex> rdLck(m_mtx);
                    if(m_curRunningTree.get()){
                        //任务结束自动进入下个
                        sts = m_curRunningTree.get()->tickOnce();
                        usleep(10000);
                        //成功执行完成后进入下阶段
                        if(NodeStatus::SUCCESS == sts){
                            status::CCommandStatus::stsTskMsg().dstwaypt = info::CParam::m_sCurNavInfo.id;
                            if(set::CSetLine::wayPts().points.size() > 0)
                                status::CCommandStatus::stsTskMsg().dstwaypt %= set::CSetLine::wayPts().points.size();
                            status::CCommandStatus::stsTskMsg().diswaypt = info::CParam::m_sCurNavInfo.dis;
                            complete = true;
                            //失败后再次执行当前任务
                        }else if(NodeStatus::FAILURE == sts){
                            tn = time(nullptr);
                            if((tn - preTm) > 15){//超过15秒再次尝试
                                if(m_ctrlViaJson){
                                    status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsFailed;
                                    status::CCommandStatus::sendTaskStatus();
                                }
                                status::CCommandStatus::cmdRspMsg().rslt="当前任务执行失败,再次尝试";
                                status::CCommandStatus::sendCmdRsp();
                                
                                preTm = tn;
                                m_curRunningTree = std::make_shared<BT::Tree>(m_factory.createTree(m_curRunningTree->subtrees[0]->tree_ID));
                            }
                            
                        }else if(NodeStatus::RUNNING == sts){//航点变换时上报正在运行状态
                            
                            if(info::CParam::m_sPreNavId != info::CParam::m_sCurNavInfo.id){//(tn - preTm > 5){
                                info::CParam::m_sPreNavId = info::CParam::m_sCurNavInfo.id;
                                status::CCommandStatus::stsTskMsg().dstwaypt = info::CParam::m_sCurNavInfo.id;
                                if(info::CParam::m_sCurStage.line.points.size() > 0)
                                    status::CCommandStatus::stsTskMsg().dstwaypt %= info::CParam::m_sCurStage.line.points.size();
                                status::CCommandStatus::stsTskMsg().diswaypt = info::CParam::m_sCurNavInfo.dis;
                                status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsOngoing;
                                status::CCommandStatus::sendTaskStatus();
                            }
                        }else if(NodeStatus::IDLE == sts){//上报未就绪状态
                            status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsNotready;
                            status::CCommandStatus::sendTaskStatus();
                        }

                    //未有执行任务进行休眠
                    }else{
                        sleep(1);
                    }
                    }catch(std::exception &e){
                        RCLCPP_WARN(rclcpp::get_logger("action_plugin_mgr"),"exception: %s so go nextstage",e.what()); 
                        return;
                    }
                }
                
                if(complete){
                    //toNxtStage(); 
                    status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsComplete;//上报任务执行完成
                    status::CCommandStatus::sendTaskStatus();                        
                }               
                
            } 
        }

        bool ActionPluginMgr::updateTree(const std::string & treeName,std::string & result){
            try{
                if(treeName.empty()){
                    result = "行为树名为空";
                    return false;
                }
                std::shared_ptr<BT::Tree> tmp = std::make_shared<BT::Tree>(m_factory.createTree(treeName));
                std::unique_lock<std::shared_timed_mutex> wtLck(m_mtx);
                m_curRunningTree = tmp;
                return true;
            }
            catch(const std::exception & e){
                result = e.what();
                return false;
            }
        }

        void ActionPluginMgr::initJsonAction(int sn){
            m_ctrlViaJson = true;
            status::CCommandStatus::stsTskMsg().stage = sn;
            status::CCommandStatus::stsTskMsg().id = info::CParam::m_sSelfId;
            status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsFailed;
            info::CParam::m_sCurNavInfo.id = -1;
            info::CParam::m_sCurNavInfo.dis = 0;
            info::CParam::m_sPreNavId = -1;
        }

    
    }
}