#ifndef ZYZN_MANAGE_TARGET_AREA_HPP
#define ZYZN_MANAGE_TARGET_AREA_HPP

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
namespace zyzn{
    namespace manage{
        /**
         *将目标区域划分为多个搜索区域（只水平区域，不考虑垂直向）
         *依据起点，计算得到此点到目标区域的最近点（作为搜索起点）
         *从搜索起点开始将目标区域划分为N个搜索区域（可按面积划分） 
         *
        */
        class CTargetArea{
            public:
            CTargetArea(const geometry_msgs::msg::Polygon area,int nums,geometry_msgs::msg::Point ptSt){

            }
            private:
            geometry_msgs::msg::Point m_ptSrch;

        };
    }
}











#endif
