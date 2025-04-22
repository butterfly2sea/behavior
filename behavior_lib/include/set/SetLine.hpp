#ifndef ZYZN_SET_LINE_HPP
#define ZYZN_SET_LINE_HPP
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <json/json.h>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/polygon.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <custom_msgs/msg/param_short.hpp>
#include <plugin/base_plugin.hpp>

using namespace BT;
namespace zyzn{
    namespace set{
        /**
         * @brief 设置航线相关信息
         * 利用对应话题发布航线、到点距离、分组、偏移等消息
         * 并发布航线飞行开始时间，可已经规定的发送内容进行发送指定项
         * @author zyzn
        */
        class CSetLine : public SyncActionNode{
            public:
            /**
             * 载具类型
            */
            enum EVehiType{
                FixWing=1, ///<!固定翼>
                Coper=2,    ///<!旋翼>
                VtolCop,     ///<!垂起-旋翼模式>
                VtolFix,     ///<!垂起-固定翼模式>
                Car          ///<!小车>
            };

            enum{
                TickCount=100
            };

            /**
             * 发送内容类型
            */
            enum SetContentTyp{
                VEHICLE_TYP=1,  ///<! 载具类型>
                SPD=2,          ///<! 速度>
                ANTI_DIS=4,     ///<! 避撞距离>
                ARV_DIS=8,      ///<! 到点判定>
                FORM=16,        ///<! 编队类型>
                GROUP=32,       ///<! 分组>
                WAY_PTS=64,     ///<! 航向航点>
                LOOPS=128,      ///<! 循环>
                IDS=256,        ///<! 分组id>
                OFFSETS=512,    ///<! 分组偏移>
                ALL = 1023,     ///<! 全部内容>
                TWO_SWITCH = IDS + OFFSETS,///<! 编队分组变化只设置同组id和偏移>
                FOUR_SWITCH = TWO_SWITCH + LOOPS + WAY_PTS///<! 编队分组变化设置同组id、偏移、循环和航点>
            };

            /**
             * json 任务中包括的参数名索引，方便获取json参数名获取
             */
            enum JsonParamIdx{
                VEHICLE_TYP_IDX=0,   ///<! 载具类型>
                SPD_IDX,             ///<! 速度>
                PT_TYP_IDX,          ///<! 航点类型>
                ARV_DIS_IDX,         ///<! 到点判定>
                WAY_PTS_IDX,         ///<! 航向航点>
                LOOPS_IDX,           ///<! 循环>
                PARAMS_COUNT
            };

            typedef std_msgs::msg::UInt8MultiArray::_data_type ids_t;
            CSetLine(const std::string & instance_name,const BT::NodeConfig& conf);
            CSetLine();

            static PortsList providedPorts();

            NodeStatus tick();
            /**
             * @brief 设置避撞距离，利用对应话题发布避撞距离消息
             * @param dis 距离值
             * @return 无
            */
            void setAntiCollDis(float dis);
            
            /**
             * @brief 设置到点距离，利用对应话题发布到点距离消息
             * @param dis 距离值
             * @return 无
            */
            void setArvDis(float dis);

            /**
             * @brief 设置编队类型，利用对应话题发布编队类型消息
             * @param typ 类型值
             * @return 无
            */
            void setForm(const custom_msgs::msg::ParamShort& typ);
            
            /**
             * @brief 设置分组，利用对应话题发布分组消息
             * @param grp 分组值
             * @return 无
            */
            void setGrp(uint8_t grp);

            /**
             * @brief 设置航点消息内容，将航线航点值通过对于话题发布，默认话题为inner/set/navline
             * @param pts 航点内容
             * @return 无
            */
            void setWayPts(const geometry_msgs::msg::Polygon& pts);

            /**
             * @brief 设置循环次数，利用对应话题发布循环次数消息
             * @param lp 分组值
             * @return 无
            */
            void setLoops(int lp);
            
            /**
             * @brief 设置速度，利用对应话题发布速度设置消息
             * @param in spd 速度值
             * @return 无
            */
            void setSpd(float spd);

            /**
             * @brief 设置载具类型，利用对应话题发布载具类型消息
             * @param typ 类型值
             * @return 无
            */
            void setVehiTyp(uint8_t typ);

            /**
             * @brief 设置排除飞机id，将偏移数组中去掉排除飞机id后的偏移赋值给消息内容
             * @param exIds 排除飞机id
             * @return 无
            */
            bool setExIds(const ids_t & exIds);
            
            /**
             * @brief 更新分组偏移
             * @param  offsets 分组内全部偏移
             * @return 无
            */
            void updateOffsets(const geometry_msgs::msg::Polygon &offsets);

            /**
             * @brief 更新分组id
             * @param  ids 分组内id
             * @return 无
            */
            void updateIds(const ids_t & ids);

            /**
             * @brief 获取排除飞机在已有分组(id数组中的索引),如排除飞机在分组中未找到则返回false
             * @param  exIds 排除飞机id
             * @param  idxs 排除飞机在分组内的索引
             * @return true:成功获取，false:失败
            */
           template <typename IdsTyp>
            static bool getExIdx(const IdsTyp & exIds,std::set<int> & idxs){
            for(const uint8_t & id:exIds){
                auto it = std::find(m_s_ids.begin(),
                m_s_ids.end(),id);
                if(m_s_ids.end() != it){
                    idxs.insert(it-m_s_ids.begin());
                }      
                        
                }
                return !idxs.empty();
            }

            /**
             * @brief 判定本机是否在分组 查看此分组的id中是否包含本机id
             * @param ids 分组id集合
             * @return true:包含,false:不包含
            */
            static bool isInGroup(const ids_t & ids);

            inline static void clearExtIds(){
                m_s_extIds.clear();
            }

            inline static int & loops(){
                return m_s_loops;
            }

            inline static int & vehiType(){
                return m_s_vehiType;
            }

            inline static float & arvDis(){
                return m_s_arvDis;
            }
            /**
             * @brief 判定编队及分组变化时是否需要设置循环次数
             * @return true:需要 false:不需
            */
            static bool isSwitchSetLoops(){
                return m_s_switchSets & WAY_PTS;
            }

            /**
             * @brief 判定编队及分组变化时是否需要设置航线
             * @return true:需要 false:不需
            */
            static bool isSwitchSetLine(){
                return m_s_switchSets & LOOPS;
            }

            /**
             * @brief 分组及编队变化设置项
             * @return void
            */
            static inline int & switchSets(){
                return m_s_switchSets;
            }

            static inline uint8_t & grp(){
                return m_s_grp;
            }

            static inline ids_t & ids(){
                return m_s_ids;
            }

            static inline geometry_msgs::msg::Polygon & offsets(){
                return m_s_offsets;
            }

            static inline geometry_msgs::msg::Polygon & wayPts(){
                return m_s_wayPts;
            }

            static inline float & spd(){
                return m_s_spd;
            }

            private:
            void init();

            /**
             * @brief 设置偏移量消息发布，如果传入的偏移量消息为空，则返回
             * @param in ofts 偏移量
            */
            void setOffsets(const geometry_msgs::msg::Polygon &ofts);

             /**
             * @brief 将新分组内飞机id赋值给消息内容进行发布
             * @param ids 新分组飞机id
             * @return 无
            */
            void setIds(const ids_t & ids);
            
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
            rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr m_pubTime;//航线飞行开始时间发布对象
            rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr m_pubLine;//航点消息发布对象
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubCollDis;//防撞距离发布对象
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubArvDis; //到点距离发布对象
            rclcpp::Publisher<custom_msgs::msg::ParamShort>::SharedPtr m_pubForm;//编队方式发布对象
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_pubGrp;//分组发布对象
            rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_pubLoops;//循环次数发布对象
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubSpd;//速度发布对象
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_pubVehiTyp;//载具类型发布对象
            rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr m_pubIds;//分组内id发布对象
            rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr m_pubOffsets;//分组偏移发布对象
            SetContentTyp m_setTyp;//待设置内容项类型
            static int m_s_loops;//循环次数
            static int m_s_vehiType;//载具类型
            static float m_s_arvDis;//到点距离

            static int m_s_switchSets;//分组、编队变化时需要设置项目，分组内id、编队偏移(循环次数和航线杨上辉要求待确定)

            static std::set<uint8_t> m_s_extIds;//排除分组id
            static uint8_t m_s_grp;//分组
            static ids_t m_s_ids;//分组内id
            static geometry_msgs::msg::Polygon m_s_offsets;//分组偏移
            static geometry_msgs::msg::Polygon m_s_wayPts;//航线点
            static float m_s_spd;//速度

        };
    }
}
#endif