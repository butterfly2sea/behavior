#include <log/Logger.hpp>
#include "../../include/set/SetRallyDst.hpp"
#include <behavior_lib/get/GetLocation.hpp>
#include <behavior_lib/info/OffboardInfo.hpp>
#include <behavior_lib/info/Param.hpp>
#include <behavior_lib/utility/Utility.hpp>
#include <behavior_lib/plugin/base_plugin.hpp>

namespace zyzn{
namespace set{

// 节点的inputs端口名，用于参数名获取
const char * CSetRallyDst::g_inputNames[CSetRallyDst::JsonParamIdx::PARAMS_COUNT]={
    "dstLocParam","pointTagParam","spdParam","vehiTypeParam","formOffsetParam","arvDisParam"
};

// 默认json任务中参数名
const char * CSetRallyDst::g_defParams[CSetRallyDst::JsonParamIdx::PARAMS_COUNT]={
    "dstLoc","pointTag","spd","vehiType","formOffset","arvDis"
};

CSetRallyDst::SRallyDstInfo CSetRallyDst::m_s_rallyDstInfo;

CSetRallyDst::CSetRallyDst(const std::string& name, const NodeConfig& config):SyncActionNode(name,config){
  updateParam();
}

PortsList CSetRallyDst::providedPorts(){
  return {
      InputPort<int>("step"),                              // 任务步骤
      InputPort<float>("obsHgh"),                         // 避障高度
      InputPort<std::string>(g_inputNames[DST_LOC_IDX]),    // 目标位置参数名
      InputPort<std::string>(g_inputNames[POINT_TAG_IDX]),  // 点类型参数名
      InputPort<std::string>(g_inputNames[SPD_IDX]),        // 速度参数名
      InputPort<std::string>(g_inputNames[VEHI_TYPE_IDX]),  // 载具类型参数名
      InputPort<std::string>(g_inputNames[FORM_OFFSET_IDX]),// 编队偏移参数名
      InputPort<std::string>(g_inputNames[ARV_DIS_IDX]),    // 到达距离参数名
      OutputPort<custom_msgs::msg::OffboardCtrl>("target") // 期望位置
  };
}

NodeStatus CSetRallyDst::tick(){
  custom_msgs::msg::OffboardCtrl ctrl;
  int step = 2; // 默认直接到目标位置
  float obsHgh = -60; // 默认避障高度
  getInput<int>("step",step);
  getInput<float>("obsHgh",obsHgh);

  // 计算最终目标点
  calculateFinalDestination();

  // 设置控制参数
  ctrl.ordmask = EOffboardMask::LocCtrl + EOffboardMask::YawCtrl;

  // 根据步骤设置不同的目标点
  switch(step) {
    case 0: // 当前水平位置障碍高度
      ctrl.x = get::CGetlocation::simpVehi().x/1e3;
      ctrl.y = get::CGetlocation::simpVehi().y/1e3;
      ctrl.z = obsHgh;
      break;
    case 1: // 目标水平位置障碍高度
      ctrl.x = m_s_rallyDstInfo.finalDstPt.x;
      ctrl.y = m_s_rallyDstInfo.finalDstPt.y;
      ctrl.z = obsHgh;
      break;
    case 2: // 目标位置
    default:
      ctrl.x = m_s_rallyDstInfo.finalDstPt.x;
      ctrl.y = m_s_rallyDstInfo.finalDstPt.y;
      ctrl.z = m_s_rallyDstInfo.finalDstPt.z;
      break;
    case 3: // 目标水平位置当前高度
      ctrl.x = m_s_rallyDstInfo.finalDstPt.x;
      ctrl.y = m_s_rallyDstInfo.finalDstPt.y;
      ctrl.z = get::CGetlocation::simpVehi().z/1e3;
      break;
  }

  // 确保z值有效
  algorithm::CUtility::checkZValid<float>(ctrl.z);

  // 如果是固定翼，使用不同的控制模式
  if(get::CGetlocation::isFixWing()){
    ctrl.ordmask = EOffboardMask::FixLocRadCtrl;
    ctrl.airspd = 0; // 使用最小空速
    ctrl.vy = 60;    // 默认盘旋半径
  } else {
    // 计算航向
    if(algorithm::CUtility::getDisFrmLoc(get::CGetlocation::simpVehi().x/1e3,
                                         get::CGetlocation::simpVehi().y/1e3, ctrl.x, ctrl.y) < 0.5){
      ctrl.yaw = get::CGetlocation::simpVehi().yaw/1e3;
    } else {
      ctrl.yaw = algorithm::CUtility::getYawN2PFrmLoc(get::CGetlocation::simpVehi().x/1e3,
                                                      get::CGetlocation::simpVehi().y/1e3, ctrl.x, ctrl.y);
    }
  }

  setOutput<custom_msgs::msg::OffboardCtrl>("target", ctrl);

  txtLog().info(THISMODULE "Rally step:%d x:%f y:%f z:%f",
                step, ctrl.x, ctrl.y, ctrl.z);

  return NodeStatus::SUCCESS;
}

void CSetRallyDst::updateParam(){
  // 获取目标位置
  const Json::Value & dstLoc = getParam(JsonParamIdx::DST_LOC_IDX);
  if(dstLoc.isArray() && dstLoc.size() > 0){
    const Json::Value & pt = dstLoc[0];
    if(pt.isMember("x_lat"))
      m_s_rallyDstInfo.baseDstPt.x = plugin::BasePlugin::getValue(pt["x_lat"]);
    if(pt.isMember("y_lon"))
      m_s_rallyDstInfo.baseDstPt.y = plugin::BasePlugin::getValue(pt["y_lon"]);
    if(pt.isMember("z_alt"))
      m_s_rallyDstInfo.baseDstPt.z = plugin::BasePlugin::getValue(pt["z_alt"]);

    algorithm::CUtility::checkZValid<float>(m_s_rallyDstInfo.baseDstPt.z);
  }

  // 获取编队偏移
  const Json::Value & formOffset = getParam(JsonParamIdx::FORM_OFFSET_IDX);
  if(formOffset.isObject()){
    if(formOffset.isMember("diff_x_lat"))
      m_s_rallyDstInfo.offset.x = plugin::BasePlugin::getValue(formOffset["diff_x_lat"]);
    if(formOffset.isMember("diff_y_lon"))
      m_s_rallyDstInfo.offset.y = plugin::BasePlugin::getValue(formOffset["diff_y_lon"]);
    if(formOffset.isMember("diff_z_alt"))
      m_s_rallyDstInfo.offset.z = plugin::BasePlugin::getValue(formOffset["diff_z_alt"]);
  }

  // 获取点类型
  const Json::Value & ptTyp = getParam(JsonParamIdx::POINT_TAG_IDX);
  m_s_rallyDstInfo.pointType = plugin::BasePlugin::Loc;
  if(!ptTyp.isNull() && ptTyp.isString()){
    m_s_rallyDstInfo.pointType = ptTyp.asString()=="loc" ? plugin::BasePlugin::Loc : plugin::BasePlugin::Gps;
  }

  // 获取速度
  m_s_rallyDstInfo.spd = plugin::BasePlugin::getValue(getParam(JsonParamIdx::SPD_IDX));

  // 获取到达距离
  m_s_rallyDstInfo.arvDis = plugin::BasePlugin::getValue(getParam(JsonParamIdx::ARV_DIS_IDX));

  // 获取载具类型
  const Json::Value & vehiTyp = getParam(JsonParamIdx::VEHI_TYPE_IDX);
  if(!vehiTyp.isNull() && vehiTyp.isString()){
    m_s_rallyDstInfo.vehiType = vehiTyp.asString();
  }

  txtLog().info(THISMODULE "Rally updateParam: baseDst(%.2f,%.2f,%.2f) offset(%.2f,%.2f,%.2f) type:%d",
                m_s_rallyDstInfo.baseDstPt.x, m_s_rallyDstInfo.baseDstPt.y, m_s_rallyDstInfo.baseDstPt.z,
                m_s_rallyDstInfo.offset.x, m_s_rallyDstInfo.offset.y, m_s_rallyDstInfo.offset.z,
                m_s_rallyDstInfo.pointType);
}

void CSetRallyDst::calculateFinalDestination(){
  // 复制基础目标点
  m_s_rallyDstInfo.finalDstPt = m_s_rallyDstInfo.baseDstPt;

  // 如果是GPS坐标，先转换为local坐标
  if(plugin::BasePlugin::Gps == m_s_rallyDstInfo.pointType){
    txtLog().info(THISMODULE "Converting GPS to local coordinates");
    algorithm::CUtility::gps2loc(info::CParam::home(), m_s_rallyDstInfo.finalDstPt);
    txtLog().info(THISMODULE "After GPS->local: (%.2f,%.2f,%.2f)",
                  m_s_rallyDstInfo.finalDstPt.x, m_s_rallyDstInfo.finalDstPt.y, m_s_rallyDstInfo.finalDstPt.z);
  }

  // 应用编队偏移
  m_s_rallyDstInfo.finalDstPt.x += m_s_rallyDstInfo.offset.x;
  m_s_rallyDstInfo.finalDstPt.y += m_s_rallyDstInfo.offset.y;
  m_s_rallyDstInfo.finalDstPt.z += m_s_rallyDstInfo.offset.z;

  txtLog().info(THISMODULE "Final destination: (%.2f,%.2f,%.2f)",
                m_s_rallyDstInfo.finalDstPt.x, m_s_rallyDstInfo.finalDstPt.y, m_s_rallyDstInfo.finalDstPt.z);
}

const Json::Value & CSetRallyDst::getParam(JsonParamIdx idx){
  if(idx < 0 || idx >= JsonParamIdx::PARAMS_COUNT)
    idx = (JsonParamIdx)0;
  std::string paramName = g_defParams[idx];
  getInput<std::string>(g_inputNames[idx], paramName);
  return info::CParam::getParam(paramName);
}
}
}