#include <fstream>
#include <algorithm>
#include <regex>
#include <boost/dll/import.hpp>
#include <log/Logger.hpp>
#include <check/CheckWayViaTp.hpp>
#include <manage/action_plugin_mgr.hpp>
#include <status/CommandStatus.hpp>
#include <set/SetLine.hpp>
#include <plugin/core_plugin.hpp>
#include "../../include/info/Param.hpp"
namespace dll = boost::dll;
namespace zyzn {
    namespace manage {
        static std::vector<dll::shared_library> g_s_libs;
        ActionPluginMgr::ActionPluginMgr() {
            m_traceAttackProc.first = UNSET;
            m_joyProc.first = UNSET;
            m_isRunning = true;
            m_trdTree = std::make_shared<std::thread>(std::bind(&ActionPluginMgr::treeRun,this));
            for(int i=0;i<ActType::CTRL_COUNT;++i)
                m_curRunningTree[i] = nullptr;
        }

        ActionPluginMgr::~ActionPluginMgr() {
            m_isRunning = false;
            m_trdTree->join();
        }

        void ActionPluginMgr::procJsonAction(const Json::Value & action,const char* cmd,int sn){
            initJsonAction(sn);
            if(!action.isMember("name")){
                status::CCommandStatus::sendTaskStatus();
                txtLog().error(THISMODULE "action name not exist");
                return;
            }
            std::string actName = action["name"].asString();
            if(m_actProc.find(actName) == m_actProc.end() || !m_actProc[actName].first){
                status::CCommandStatus::sendTaskStatus();
                txtLog().error(THISMODULE "action:%s not surpport",actName.c_str());
                return;
            }
            if(!action.isMember("params") || !action["params"].isArray()){
                status::CCommandStatus::sendTaskStatus();
                txtLog().error(THISMODULE "action:%s params not exist or not array",actName.c_str());
                return;
            }
            const Json::Value & params = action["params"];
            if(!m_actProc[actName].first->parseJsonParam(params)){
                status::CCommandStatus::sendTaskStatus();
                txtLog().error(THISMODULE "action:%s params parse error",actName.c_str());
                return;
            }
            Json::Value triggers;
            if(action.isMember("triggers")){
                triggers = action["triggers"];
            }
            info::CParam::actionName() = actName;
            saveParamAndTrigger(params,triggers);
            std::string treeName(actName);
            treeName += "-";
            treeName += cmd;
            if(updateTree(treeName,status::CCommandStatus::cmdRspMsg().rslt,m_actProc[actName].second))
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
            std::string actName;
            ActType actTyp = ActType::CTRL_VEHICLE;
            getActInfo(treeName,actName,actTyp);
            return updateTree(actName,result,actTyp);
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
            std::string actName;
            ActType actTyp = ActType::CTRL_VEHICLE;
            getActInfo(treeName,actName,actTyp);
            return updateTree(actName,result,actTyp);
        }

        bool ActionPluginMgr::loadPlugin(const std::string & path,const std::string & name) {
            std::ifstream ifs;
            bool addDirSep = path.find_last_of("/")!=path.length()-1;
            std::string treePath=(addDirSep?"/":"");
            treePath += "tree/";
            std::string configPath = path + treePath + name;
            ifs.open(configPath,std::ifstream::in);
            if(!ifs.is_open()){
               txtLog().error(THISMODULE "open file error");
                return false;
            }
            Json::Value root;
            Json::CharReaderBuilder builder;
            Json::String err;
            if(!Json::parseFromStream(builder,ifs,&root,&err)){
                txtLog().error(THISMODULE "parse json error");
                return false;
            }
            if(root["plugins"].isNull() || root["plugins"].isArray() == false){
                txtLog().error(THISMODULE "parse plugins json error");
                return false;
            }
            if(root["xmlFiles"].isNull() || root["xmlFiles"].isArray() == false){
                txtLog().error(THISMODULE "parse xmlFiles json error");
                return false;
            }
            m_plugins.reserve(10);
            g_s_libs.reserve(10);
            std::shared_ptr<plugin::BasePlugin> corePlugin = std::make_shared<plugin::CorePlugin>();//创建coreplugin
            corePlugin->registerNode(m_factory);
            m_plugins.push_back(corePlugin);
            if(root.isMember("coreActions") && root["coreActions"].isArray()){//添加coreplugin支持的配置行为
                const Json::Value & coreActions = root["coreActions"];
                for(const Json::Value & coreAct : coreActions){
                    std::string actName;
                    ActType actTyp = ActType::CTRL_VEHICLE;
                    getActInfo(coreAct.asString(),actName,actTyp);
                    m_actProc[actName].first = corePlugin;
                    m_actProc[actName].second = actTyp;
                }
            }
            //添加其它插件
            Json::Value & plugins = root["plugins"];
            for(const Json::Value & plugin : plugins){
                if(!plugin.isMember("name"))
                    continue;
                std::string pluginName = path + treePath + plugin["name"].asString();
                dll::shared_library lib(pluginName, dll::load_mode::append_decorations);//加载库
                
                if (!lib.has("create_plugin")) 
                    continue;
                g_s_libs.push_back(lib);
                using plugin_create_t = PluginPtr_t ();
                auto creator = dll::import_alias<plugin_create_t>(
                    g_s_libs[g_s_libs.size()-1], "create_plugin"
                );
                PluginPtr_t plg = creator();//创建插件
                plg->registerNode(m_factory);//调用插件的行为节点注册函数
                m_plugins.push_back(plg);
                if(!plugin.isMember("actions") || !plugin["actions"].isArray())
                    continue;
                const Json::Value & acts = plugin["actions"];//获取插件支持的配置行为
                for(const Json::Value & act:acts){
                    std::string actName;
                    ActType actTyp = ActType::CTRL_VEHICLE;
                    getActInfo(act.asString(),actName,actTyp);
                   
                    //将对应行为映射到对应插件
                    m_actProc[actName].first = plg;
                    m_actProc[actName].second = actTyp;
                }
            }
            Json::Value & xmlFiles = root["xmlFiles"];//获取全部行为树xml文件
            for(const Json::Value & xmlFile:xmlFiles){
                std::string xmlName = path + treePath + xmlFile.asString();
                m_factory.registerBehaviorTreeFromFile(xmlName);//从行为树xml文件注册行为树
            }
            ifs.close();
            return true;
        }

        void ActionPluginMgr::treeRun(){
            time_t preTm = time(nullptr);
            time_t tn;
            
            while(m_isRunning ){
                NodeStatus sts = NodeStatus::RUNNING;
                {
                    try{
                        std::shared_lock<std::shared_timed_mutex> rdLck(m_mtx);
                        if(!hasValidTree()){
                            sleep(1);
                        }
                        //其它行为树
                        for(int idx=ActType::CTRL_POD;idx<ActType::CTRL_COUNT;++idx){
                            if(m_curRunningTree[idx]){
                                sts = m_curRunningTree[idx].get()->tickOnce();
                                if(sts == NodeStatus::SUCCESS || sts == NodeStatus::FAILURE){
                                    m_curRunningTree[idx] = nullptr;
                                }else{
                                    usleep(1000);
                                }
                                
                            }
                        }
                        //控制飞机的行为树
                        if(!m_curRunningTree[ActType::CTRL_VEHICLE]){
                            continue;
                        }
                        //任务结束自动进入悬停
                        sts = m_curRunningTree[ActType::CTRL_VEHICLE].get()->tickOnce();
                        usleep(1000);
                        //成功执行完成后进入下阶段
                        if(NodeStatus::SUCCESS == sts){
                            status::CCommandStatus::stsTskMsg().dstwaypt = check::CCheckWayViaTp::curNav().id;
                            if(set::CSetLine::wayPts().points.size() > 0)
                                status::CCommandStatus::stsTskMsg().dstwaypt %= set::CSetLine::wayPts().points.size();
                            status::CCommandStatus::stsTskMsg().diswaypt = check::CCheckWayViaTp::curNav().dis;
                            m_curRunningTree[ActType::CTRL_VEHICLE] = std::make_shared<BT::Tree>(m_factory.createTree("LoitTree"));
                            status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsComplete;//上报任务执行完成
                            status::CCommandStatus::sendTaskStatus();    
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
                                m_curRunningTree[ActType::CTRL_VEHICLE] = std::make_shared<BT::Tree>(m_factory.createTree(m_curRunningTree[ActType::CTRL_VEHICLE]->subtrees[0]->tree_ID));
                            }
                            
                        }else if(NodeStatus::RUNNING == sts){//航点变换时上报正在运行状态
                            if(check::CCheckWayViaTp::preNavId() != check::CCheckWayViaTp::curNav().id){//(tn - preTm > 5){
                                check::CCheckWayViaTp::preNavId() = check::CCheckWayViaTp::curNav().id;
                                status::CCommandStatus::stsTskMsg().dstwaypt = check::CCheckWayViaTp::curNav().id;
                                status::CCommandStatus::stsTskMsg().diswaypt = check::CCheckWayViaTp::curNav().dis;
                                status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsOngoing;
                                status::CCommandStatus::sendTaskStatus();
                            }
                        }else if(NodeStatus::IDLE == sts){//上报未就绪状态
                            status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsNotready;
                            status::CCommandStatus::sendTaskStatus();
                        }
                    }
                    catch(std::exception &e){
                        txtLog().warnning(THISMODULE "exception: %s so go nextstage",e.what()); 
                        return;
                    }
                }          
                
            } 
        }

        bool ActionPluginMgr::updateTree(const std::string & treeName,std::string & result,ActType typ){
            try{
                if(typ < 0 || typ >= ActType::CTRL_COUNT){
                    txtLog().error(THISMODULE "行为控制类型不支持 name:%s",treeName.c_str());
                    result = "行为控制类型不支持";
                    return false;
                }
                if(treeName.empty()){
                    result = "行为树名为空";
                    txtLog().error(THISMODULE "行为树名为空");
                    return false;
                }
                std::shared_ptr<BT::Tree> tmp = std::make_shared<BT::Tree>(m_factory.createTree(treeName));
                std::unique_lock<std::shared_timed_mutex> wtLck(m_mtx);
                m_curRunningTree[typ] = tmp;
                return true;
            }
            catch(const std::exception & e){
                result = e.what();
                txtLog().error(THISMODULE "updateTree exception: %s",e.what());
                return false;
            }
        }

        void ActionPluginMgr::initJsonAction(int sn){
            m_ctrlViaJson = true;
            status::CCommandStatus::stsTskMsg().stage = sn;
            status::CCommandStatus::stsTskMsg().id = info::CParam::vehiId();
            status::CCommandStatus::stsTskMsg().status = status::CCommandStatus::EStatusStg::StsFailed;
            check::CCheckWayViaTp::curNav().id = -1;
            check::CCheckWayViaTp::curNav().dis = 0;
            check::CCheckWayViaTp::preNavId() = -1;
        }

        void ActionPluginMgr::saveParamAndTrigger(const Json::Value & params,const Json::Value & triggers){
            for(const Json::Value & param:params){
                if(param.isObject() && param.isMember("name") && param.isMember("value")){
                    info::CParam::updateParam(param["name"].asCString(),param["value"]);
                }
            }
            if(!triggers.isArray()){
                return;
            }
            for(const Json::Value & trigger:triggers){
                if(trigger.isObject() && trigger.isMember("name") && trigger.isMember("value")){
                    info::CParam::updateTrigger(trigger["name"].asCString(),trigger["value"]);
                }
            }

        }

        void ActionPluginMgr::getActInfo(std::string actInfo,std::string & actName,ActType & actTyp){
            char* typ = strrchr((char *)actInfo.c_str(),'-');
            actName = actInfo;
            actTyp = ActType::CTRL_VEHICLE;
            if(!typ){
                return;
            }
            std::string strTyp(typ+1);
            strTyp.erase(std::remove(strTyp.begin(),strTyp.end(),' '),strTyp.end());
            std::regex reg("^[-+]?\\d+$");
            if(!std::regex_match(strTyp, reg)){
                return;
            }
            
            try{
                int type = atoi(strTyp.c_str());
                if(type >=0 && type < ActType::CTRL_COUNT){
                    actName.resize(actInfo.length()-strlen(typ));
                    actTyp = (ActType)type;
                    return;
                }
            }
            catch(const std::exception& e){
                return;
            }
             
        }
    
    }
}