#include "../../include/control/CameraCtrl.hpp"
#include <behavior_lib/info/Param.hpp>
namespace zyzn{
    namespace ctrl{

        PortsList CCameraCtrl::providedPorts(){
            return {InputPort<int>("ctrltyp"),//相机控制消息中控制类型
            InputPort<int>("param1"),//相机控制消息中控制参数
            InputPort<int>("param2"),//相机控制消息中控制参数
            InputPort<int>("param3"),//相机控制消息中控制参数
            InputPort<int>("intval")//相机控制2次控制间隔毫秒
            };
        }
        CCameraCtrl::CCameraCtrl(const std::string & name,
        const NodeConfig & config):SyncActionNode(name,config),
            m_param1(InvalDeg),
            m_param2(InvalDeg),
            m_param3(InvalDeg),
            m_typ(Inval),
            m_ts(0),
            m_fltMs(3000),
            m_camCtrl(nullptr){
                m_camCtrl = info::CParam::rosNode()->create_publisher<custom_msgs::msg::CameraCtrl>("inner/camera/control",rclcpp::SensorDataQoS());
                getInput<int>("intval",m_fltMs);
        }

        bool CCameraCtrl::filter(){
                int ctrltyp = Inval;
                getInput<int>("ctrltyp",ctrltyp);
                if(Inval == ctrltyp)
                    return false;
                int64_t tNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                if(UnTrack == (CtrlTyp)ctrltyp){//如当前为取消凝视则先不赋值为ctrltyp，等取消凝视后再赋值
                    m_ts = tNow;
                    m_typ = ctrltyp;
                    return true;
                }
                if((tNow-m_ts)<m_fltMs)
                    return false;
                
                if(Cent == ctrltyp && Cent == m_typ)//忽略连续回中
                    return false;
                int param1 = m_param1;
                int param2 = m_param2;
                int param3 = m_param3;
                getInput<int>("param1",param1);
                getInput<int>("param2",param2);
                getInput<int>("param3",param3);

                if((Atti == ctrltyp) &&(Atti == m_typ)){
                    //连续的姿态控制如果前后2次角度差别小于1则忽略本次
                    //连续的gps凝视如果前后位置一致则忽略本次
                    if(abs(param1-m_param1)<=1 && abs(param2-m_param2)<=1)
                        return false;
                }
                if((Track == ctrltyp) &&(Track == m_typ)){
                    //连续的凝视如果前后2次像素差别小于3则忽略本次
                    if(abs(param1-m_param1)<=3 && abs(param2-m_param2)<=3)
                        return false;
                }

                /*
                if((GpsTrack == (CtrlTyp)ctrltyp) && (GpsTrack == (CtrlTyp)m_typ)){//gps凝视
                    
                    if((param1 == m_param1) && (param2 == m_param2)){
                        return false;
                    }
                }
                */
    
                m_typ = ctrltyp;
                m_ts = tNow;
                m_param1 = param1;
                m_param2 = param2;
                m_param3 = param3;
                return true;
            }

            NodeStatus CCameraCtrl::tick(){
                if(!filter())
                    return NodeStatus::FAILURE; 
                m_msg.ctrl_type = m_typ;   
                switch (m_typ)
                {
                case Cent://回中控制给相机的指令为姿态控制
                    m_msg.ctrl_type = Atti;
                    m_msg.others.resize(1);
                    break;
                case Atti:
                    m_msg.others.resize(5);
                    m_msg.others[0] = 0;
                    m_msg.others[1] = 1;
                    m_msg.others[2] = m_param1;
                    m_msg.others[3] = 1;
                    m_msg.others[4] = m_param2;
                    //m_msg.others[4] -= 45;
                    if(m_msg.others[4] < -45){
                        m_msg.others[4] = -45;
                    }else if(m_msg.others[4]>90){
                        m_msg.others[4] = 90;
                    }
                    m_msg.others[2] *= 100;
                    m_msg.others[4] *= 100;
                    break;
                case Track:
                    m_msg.others.resize(2);
                    m_msg.others[0] = m_param1;
                    m_msg.others[1] = m_param2;
                    break;
                case GpsTrack:
                    m_msg.others.resize(3);
                    m_msg.others[0] = m_param1;
                    m_msg.others[1] = m_param2;
                    m_msg.others[2] = m_param3;
                    break;
                case UnTrack:
                    break;
                default:
                    return NodeStatus::FAILURE;
                    break;
                }
                m_camCtrl->publish(m_msg);
                return NodeStatus::SUCCESS;
            }
    }
}