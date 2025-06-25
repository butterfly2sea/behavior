#ifndef ZYZN_MANAGE_SEARCH_AREA_HPP
#define ZYZN_MANAGE_SEARCH_AREA_HPP

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace zyzn{
    namespace manage{
        /**
         *从区域（目前只考虑比较简单的多边形区域，起点总是在最南、北、东或西）计算得出搜索航线
	     *区域的第一个点为搜索的起点
         *如果区域在起点的北边，则从南到北进行（按扫描间隔北移）
         *区域在起点南边，则从北到南（按扫描间隔南移）
         *依次如上
         * 
        */
        class CSearchArea{
            public:
            enum ESrchDir{
                North=0,
                South,
                East,
                West,
                Mid,
                Unknow=-1
            };

            /**
             * area:搜索区域
             * schIntval:扫描间隔
            */
            CSearchArea(const geometry_msgs::msg::Polygon area,float schIntval):m_schDir(Unknow),
            m_dis(0){
                updateLine(area,schIntval);
            }
            
            /**获取搜索航线
            */
            const geometry_msgs::msg::Polygon & schLine(){
                return m_schLine;
            }

            /**
             * 获取航线总距离长度
            */
            float schDis(){
                return m_dis;
            }

            /**
             * 获取扫描方向,起点在最南则扫描方向为北
             * 如同时最南、最西则取北值（南北优先）
            */
            ESrchDir schDir(){
                return m_schDir;
            }

            private:
            /**
             * 依据区域及扫描间隔更新航线、航线总长度距离及扫描方向
            */
            void updateLine(const geometry_msgs::msg::Polygon area,float schIntval){

            }
            
            private:
            geometry_msgs::msg::Polygon m_schLine; //搜索航线
            ESrchDir m_schDir;                     //扫描方向
            float m_dis;                           //航线总距离长度

        };
    }
}











#endif
