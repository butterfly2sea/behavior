#ifndef ZYZN_MANAGE_TRACE_AREA_HPP
#define ZYZN_MANAGE_TRACE_AREA_HPP
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "../../include/utility/Utility.hpp"
namespace bg = boost::geometry;
namespace zyzn{
    namespace manage{
        class CTraceArea{
            public:
            typedef float ptVal_t;
            typedef bg::model::d2::point_xy<float> Point;
            typedef bg::model::polygon<Point, false> Polygon;
           
            
            CTraceArea(){

            }

            /**
             * @brief
             * @details
             * @param in num 区域中航线划分数量
             * @param in ptC 中点位置(斜率向黄金分割点位置)
             * @param in wid 区域宽度(y向)
             * @param in hgh 区域高度(x向)
             * @param in vx  目标速度x分量
             * @param in vy  目标速度y分量
             * @param in itv 间隔(区域划分水平间隔)
            */
            bool update(int num,const geometry_msgs::msg::Point32 &ptC,CTraceArea::ptVal_t wid,
            CTraceArea::ptVal_t hgh,CTraceArea::ptVal_t vx,CTraceArea::ptVal_t vy,
            CTraceArea::ptVal_t itv){
                if(0==itv || 0==num){
                    return false;
                }
                m_lines.clear();
                int scanNum = hgh/itv;//扫描次数(每次扫描由2点构成)
                //获取旋转角度()
                float ang = algorithm::CUtility::getRadFrmLoc(0,0,vx,vy);
                //获取平移量
                float mx = ptC.x - hgh/2;
                float my = ptC.y - wid*0.382;//长度上使用黄金分割点比例
                float z = ptC.z;
                updateArea(wid,hgh,ang,mx,my);
   
                int scansPerVehi = scanNum/num;
                if(scansPerVehi <= 0)
                    scansPerVehi = 1;
                geometry_msgs::msg::Point32 pt1;
                geometry_msgs::msg::Point32 pt2;
                pt1.z=pt2.z=ptC.z;
                for(int vehiIdx=0;vehiIdx<num;++vehiIdx){
                    int scanSt = vehiIdx*scansPerVehi;
                    int idx = 1;
                    while(idx <= scansPerVehi){
                        //获取每次扫描的2个点，然后进行旋转及平移计算得出最终位置
                        int scanIdx = (scanSt+idx);
                        if(scanIdx > scanNum){
                            scanIdx = scanNum;
                        }
                        float x1,x2,y1=0,y2=wid;
                        x1=x2= scanIdx*itv;
                        algorithm::CUtility::getRotaxy(x1,y1,ang,x1,y1);
                        algorithm::CUtility::getRotaxy(x2,y2,ang,x2,y2);
                        //平移
                        x1+=mx;
                        y1+=my;
                        x2+=mx;
                        y2+=my;
                        pt1.x=x1;
                        pt1.y=y1;
                        pt2.x=x2;
                        pt2.y=y2;
                        if(idx%2==0){//先放x2 y2
                            m_lines[vehiIdx].points.push_back(pt2);
                            m_lines[vehiIdx].points.push_back(pt1);
                        }else{//先放x1 y1
                            m_lines[vehiIdx].points.push_back(pt1);
                            m_lines[vehiIdx].points.push_back(pt2);                           
                        }
                        ++idx;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("TraceArea"),"vehiidx:%d",vehiIdx);
                    for_each(m_lines[vehiIdx].points.begin(),m_lines[vehiIdx].points.end(),[&](geometry_msgs::msg::Point32& pt){
                        RCLCPP_INFO(rclcpp::get_logger("TraceArea"),"x:%f y:%f",pt.x,pt.y);
                    });

                    
                }                
            }

            inline const geometry_msgs::msg::Polygon& getLine(int idx){
                return m_lines[idx];
            }

            inline bool isInArea(float x,float y){
                return bg::within(Point(y,x),m_area);
            }


            private:
            void updateArea(const CTraceArea::ptVal_t &wid,const CTraceArea::ptVal_t & hgh,
            const CTraceArea::ptVal_t& ang,
            const CTraceArea::ptVal_t & mx,const CTraceArea::ptVal_t & my){
                m_area.clear();
                float x[4]={0,hgh,hgh,0};
                float y[4]={0,0,wid,wid};
                RCLCPP_INFO(rclcpp::get_logger("TraceArea"),"wid:%f hgh:%f mx:%f my:%f ang:%f",
                wid,hgh,mx,my,ang);
                for(int i=0;i<4;++i){
                    algorithm::CUtility::getRotaxy(x[i],y[i],ang,x[i],y[i]);
                    x[i] += mx;
                    y[i] += my;
                    Point pt(y[i],x[i]);
                    m_area.outer().push_back(pt);
                    RCLCPP_INFO(rclcpp::get_logger("TraceArea"),"pt[%d] x:%f y:%f",i,x[i],y[i]);
                }
                m_area.outer().push_back(Point(y[0],x[0]));

            }

            private:
            std::map<int,geometry_msgs::msg::Polygon> m_lines;//存放全部索引的航线
            Polygon m_area;
        };
    }
}

#endif