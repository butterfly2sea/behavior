#ifndef ZYZN_STATUS_TASK_HPP
#define ZYZN_STATUS_TASK_HPP

#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <custom_msgs/msg/status_task.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>


using namespace BT;
namespace zyzn{
    namespace status{
        class CTaskStatus : public RosTopicPubNode<custom_msgs::msg::StatusTask>{
            public:
            CTaskStatus(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params):RosTopicPubNode<custom_msgs::msg::StatusTask>(name, conf, params){

            }

            static PortsList providedPorts()
            {
                return providedBasicPorts({InputPort<int>("id"),
                InputPort<int>("sn"),
                InputPort<int>("tag"),
                InputPort<int>("status"),
                InputPort<int>("wayid"),
                InputPort<int>("waydis"),
                InputPort<int>("objid"),
                InputPort<int>("objdis")});
            }

            static void regist(BehaviorTreeFactory &factory,RosNodeParams &param){
                param.default_port_value = "outer/information/status_task";
                factory.registerNodeType<CTaskStatus>("TaskStatus",param);
            }
         
            bool setMessage(custom_msgs::msg::StatusTask& msg){
                int id=0;
                int stage = 0;
                int tag = 0;
                int status = 0;
                int diswy=0;
                int wyid = 0;
                int obj = 0;
                int disobj = 0;
                getInput<int>("id",id);
                getInput<int>("sn",stage);
                getInput<int>("tag",tag);
                getInput<int>("status",status);
                getInput<int>("wayid",wyid);
                getInput<int>("waydis",diswy);
                getInput<int>("objid",obj);
                getInput<int>("objdis",disobj);
                msg.id = id;
                msg.stage = stage;
                msg.tag = tag;
                msg.status = status;
                msg.diswaypt = diswy;
                msg.dstwaypt = wyid;
                msg.obj = obj;
                msg.disobj = disobj;
                
                return true;
            }
            private:
        };
    }
}

#endif