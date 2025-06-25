#include "../../include/control/Joystick.hpp"
#include "../../include/info/Param.hpp"
namespace zyzn{
    namespace ctrl{
        bool CJoystick::m_s_startJoy = false;
        CJoystick::CJoystick(const std::string& name, const NodeConfig& config):SyncActionNode(name,config),
            m_subJoy(nullptr),
            m_pubManCtrl(nullptr),
            m_rcvTs(InvalTs),
            m_lostLen(LostCtrlLen),
            m_pitchCh(PitchCh),
            m_rollCh(RollCh),
            m_throtCh(ThroCh),
            m_yawCh(YawCh){
                m_s_startJoy = true;
                m_pubManCtrl = info::CParam::m_glbNode->create_publisher<mavros_msgs::msg::ManualControl>("/mavros/manual_control/send",1);
                m_subJoy = info::CParam::m_glbNode->create_subscription<sensor_msgs::msg::Joy>("/inner/control/joystick",rclcpp::SensorDataQoS(),std::bind(&CJoystick::joyCB,
                this,std::placeholders::_1));
                m_rcvTs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                getInput<int>("lost",m_lostLen);
                getInput<int>("pch",m_pitchCh);
                getInput<int>("rch",m_rollCh);
                getInput<int>("tch",m_throtCh);
                getInput<int>("ych",m_yawCh);
            }
           
            PortsList CJoystick::providedPorts()
            {
                return { InputPort<int>("lost")};
     
            }
            NodeStatus CJoystick::tick(){
                
                int64_t tNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                if((tNow-m_rcvTs) > m_lostLen){
                    m_s_startJoy = false;
                    return NodeStatus::FAILURE;
                }
                m_pubManCtrl->publish(m_ctrl);
                return NodeStatus::SUCCESS;
            }

            void CJoystick::joyCB(const sensor_msgs::msg::Joy::SharedPtr joy){
                joy2Man(joy->buttons[m_pitchCh],m_ctrl.x);
                joy2Man(joy->buttons[m_rollCh],m_ctrl.y);
                joy2Man(joy->buttons[m_throtCh],m_ctrl.z);
                joy2Man(joy->buttons[m_yawCh],m_ctrl.r);
                m_rcvTs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            }
            
    }
}