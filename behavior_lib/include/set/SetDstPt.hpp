#ifndef ZYZN_SET_DSTPT_HPP
#define ZYZN_SET_DSTPT_HPP
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/point32.hpp>
#include <json/json.h>
using namespace BT;

namespace zyzn{
    namespace set{  

        /**
         * @brief 设置目的点
         * @details 现在主要用于详查时，先飞到详查点（此目的点）
         * @author zyzn
        */
        class CSetDstPt : public BT::SyncActionNode{
            public:
            enum EStep{
                CurHorObsHgh=0,    ///<! 当前水平位置 障碍高度>
                DstHorObsHgh,      ///<! 目的水平位置 障碍高度>
                DstPos,            ///<! 目的位置>
                DstHorCurHgh       ///<! 目的水平位置 当前高度>
            };
            /**
             * 详查点信息
            */
            struct SAutoDstPt{
                geometry_msgs::msg::Point32 dstPt;
                float rdis;
                float intval;
                float alt;
                int type;
            };       

            enum{
                ObsHgh=-60         ///<! 障碍高度z值，默认60米>
            };

            /**
             * json 任务中包括的参数名索引，方便获取json参数名获取
             */
            enum JsonParamIdx{
                RDS_IDX=0,           ///<! 半径>
                ITV_IDX,             ///<! 间隔>
                ALT_IDX,             ///<! 高度>
                PT_TYP_IDX,          ///<! 点类型>
                DST_IDX,             ///<! 目标位置>
                PARAMS_COUNT
            };

            CSetDstPt(const std::string& name, const NodeConfig& config);

            /**
             * @brief 输入输出参数,节点提供的输入输出参数名及类型申明
             * target：用于offboard控制时提供位置信息
             * @result 输入输出参数列表
            */
            static PortsList providedPorts();

            inline static SAutoDstPt & dstPt(){
                return m_s_autoDstPt;
            }
                        
            /**
             * @brief 定时回调,将地面站给出的详查位置提供给offboard控制输出参数
             * @result 总算返回SUCCESS
            */
            NodeStatus tick();    
            private:
            /**
             * @brief 更新参数信息 从任务参数中获取使用的参数信息
             */
            void updateParam();

            /**
             * @brief 依据参数索引获取参数值
             * @param idx 索引
             * @return 返回对应保存在CParam中的参数值
             */
            const Json::Value & getParam(JsonParamIdx idx);

            private:
            static SAutoDstPt m_s_autoDstPt;//详查点信息
                    
        };
    }
}

#endif