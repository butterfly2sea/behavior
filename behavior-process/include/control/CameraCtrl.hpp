#ifndef ZYZN_CONTROL_CAMERACTRL_HPP
#define ZYZN_CONTROL_CAMERACTRL_HPP
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <custom_msgs/msg/camera_ctrl.hpp>


using namespace BT;
namespace zyzn{
    namespace ctrl{
        class CCameraCtrl : public SyncActionNode{
            public:
            enum CtrlTyp{
                Inval = -1,
                Track = 122,    ///<! 像素凝视>
                UnTrack = 123,  ///<! 取消凝视>
                GpsTrack = 124, ///<! gps凝视>
                Atti = 4,       ///<! 姿态>
                Cent = 0        ///<! 回中>
            };

            enum{
                InvalDeg = -9999
            };

            static PortsList providedPorts();

            CCameraCtrl(const std::string & name,
            const NodeConfig & config);

            NodeStatus tick();
            private:
            bool filter();
            private:
            int m_param1;
            int m_param2;
            int m_param3;
            int m_typ;
            int64_t m_ts;//上次发送时间
            int m_fltMs;//2次发送间隔时间毫秒
            rclcpp::Publisher<custom_msgs::msg::CameraCtrl>::SharedPtr m_camCtrl;//相机控制
            custom_msgs::msg::CameraCtrl m_msg;//相机控制消息
        };
    }
}

#endif