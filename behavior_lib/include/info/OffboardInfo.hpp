#ifndef ZYZN_INFO_OFFBOARD_HPP
#define ZYZN_INFO_OFFBOARD_HPP
#include <string_view>
#include <custom_msgs/msg/offboard_ctrl.hpp>
namespace BT{
        /**
         * offboard 控制量参数类型
        */
        struct SOffboardCtrl{
        unsigned short ordMask;//控制量标识，为EOffboardMask复合值
        float roll;     //横滚
        float pitch;    //俯仰
        float yaw;      //偏航
        float throttle; //油门
        float x;        //locx
        float y;        //locy
        float z;        //locz
        float vx;       //loc vx
        float vy;       //loc vy
        float vz;       //loc vz
        float airSpd;   //空速
        float lon;      //经度
        float lat;      //纬度
        float alt;      //海拔高度
    };

    /**
     * @brief实现和参数互相转换
     * @details
    */
    template <> 
    inline custom_msgs::msg::OffboardCtrl BT::convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 15)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            custom_msgs::msg::OffboardCtrl output;
            
            output.ordmask= convertFromString<unsigned short>(parts[0]);
            output.roll = convertFromString<float>(parts[1]);
            output.pitch = convertFromString<float>(parts[2]);
            output.yaw = convertFromString<float>(parts[3]);
            output.throttle = convertFromString<float>(parts[4]);

            output.x = convertFromString<float>(parts[5]);
            output.y = convertFromString<float>(parts[6]);
            output.z = convertFromString<float>(parts[7]);

            output.vx = convertFromString<float>(parts[8]);
            output.vy = convertFromString<float>(parts[9]);
            output.vz = convertFromString<float>(parts[10]);

            output.airspd = convertFromString<float>(parts[11]);
            output.lon = convertFromString<float>(parts[12]);
            output.lat = convertFromString<float>(parts[13]);
            output.alt = convertFromString<float>(parts[14]);
            return output;
        }
    }

    /**
     * offboard控制字
    */
    enum EOffboardMask{
        RollCtrl=1,
        PitchCtrl=2,
        YawCtrl=4,
        ThrtCtrl=8,
        LocCtrl=0x70,//16|32|64
        VxCtrl=128,
        VyCtrl=256,
        VzCtrl=512,
        VelCtrl=0x0380,//128|256|512,
        AttiCtrl=0x0F,//1|2|4|8,
        AirSpdCtrl=1024,
        GpsCtrl=0x3800,//2048|4096|8192
        FixWingCtrl,
        FixLocRadCtrl = LocCtrl + VyCtrl + AirSpdCtrl ///<! 固定翼盘旋控制>
    };

    enum{
        MicRadPerDeg = 17,///<!每度的豪弧值>
        YawErrDeg = 10,   ///<!判定到达目标航向的误差,度数>
        YawErrRad = YawErrDeg*MicRadPerDeg,///<!判定到达目标航向的误差,豪弧数>
        InitYaw=9999///<!航向初始量用于判定偏航是否设置>
    };

}  

#endif

   