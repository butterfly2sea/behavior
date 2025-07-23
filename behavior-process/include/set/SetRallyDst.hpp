#ifndef ZYZN_SET_RALLYDST_HPP
#define ZYZN_SET_RALLYDST_HPP
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/point32.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <json/json.h>
using namespace BT;

namespace zyzn{
namespace set{

/**
 * @brief 设置Rally集结目的点
 * @details 解析JSON中的集结任务参数，计算本机的目标位置（包含编队偏移）
 * @author zyzn
*/
class CSetRallyDst : public BT::SyncActionNode{
 public:

  /**
   * Rally集结点信息
  */
  struct SRallyDstInfo{
    geometry_msgs::msg::Point32 baseDstPt;  // 基础目标点
    geometry_msgs::msg::Point32 offset;     // 编队偏移
    geometry_msgs::msg::Point32 finalDstPt; // 最终目标点（基础点+偏移）
    float spd;                               // 飞行速度
    float arvDis;                           // 到达距离
    int pointType;                          // 点类型：0=GPS, 1=LOC
    std::string vehiType;                   // 载具类型
    std::string triggerType;                // 触发类型
  };

  /**
   * json 任务中包括的参数名索引
   */
  enum JsonParamIdx{
    DST_LOC_IDX=0,      // 目标位置
    POINT_TAG_IDX,      // 点类型标签
    SPD_IDX,            // 速度
    VEHI_TYPE_IDX,      // 载具类型
    FORM_OFFSET_IDX,    // 编队偏移
    ARV_DIS_IDX,        // 到达距离
    PARAMS_COUNT
  };

  CSetRallyDst(const std::string& name, const NodeConfig& config);

  /**
   * @brief 输入输出参数
   * target：用于offboard控制时提供位置信息
   * @result 输入输出参数列表
  */
  static PortsList providedPorts();

  /**
   * @brief 获取Rally目标信息
   */
  inline static SRallyDstInfo & rallyDstInfo(){
    return m_s_rallyDstInfo;
  }

  /**
   * @brief 定时回调，计算并设置Rally集结目标点
   * @result 总是返回SUCCESS
  */
  NodeStatus tick();

 private:
  /**
   * @brief 更新参数信息，从JSON任务参数中获取Rally相关参数
   */
  void updateParam();

  /**
   * @brief 计算最终目标点（基础点+编队偏移）
   */
  void calculateFinalDestination();

  /**
   * @brief 依据参数索引获取参数值
   * @param idx 索引
   * @return 返回对应保存在CParam中的参数值
   */
  const Json::Value & getParam(JsonParamIdx idx);

 private:
  static SRallyDstInfo m_s_rallyDstInfo;  // Rally目标信息
  static const char* g_inputNames[PARAMS_COUNT];   // 输入端口名
  static const char* g_defParams[PARAMS_COUNT];    // 默认参数名

};
}
}

#endif