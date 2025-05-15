#ifndef UTILITY_ZYZN_ALGORITHM_H
#define UTILITY_ZYZN_ALGORITHM_H
#include<math.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;

#include<vector>
#include<geometry_msgs/msg/point.hpp>
#include<geometry_msgs/msg/point32.hpp>
#include<geo/geo.h>

namespace zyzn
{
    namespace algorithm
    {

     /**
     * @brief 工具类
     * @details 提供角度弧度转换、速度计算
     * @author zyzn
    */    
    class CUtility
    {
        public:
        enum {
            ER = 6378140
        };
        typedef bg::model::d2::point_xy<float> Point;
        typedef bg::model::polygon<Point, false> Polygon;
        
        enum EDefVal{
            DefZ=-50,
            VehiDis=10,
            RscZ = -30,
            DtcZ = -20,
            InvalidAlt = -99999 ///<! 无效海拔值>
        };
        
        inline static double ang2rad(double ang){
            
            return M_PI * ang / 180;
        }

        inline static double rad2ang(double rad){
            return 180 * rad / M_PI;
        }

        /**
         * @brief 依据纬度差计算locx差值，只简单使用地球半径和弧度角度值计算
         * @param diffLat 纬度差值，度数
         * @return locx的差值
        */
        inline static double getDiffX(double diffLat){
            return ang2rad(diffLat) * ms_radius;
        }

        /**
         * @brief 依据当前弧度和目的弧度计算速度值（弧度）
         * 只简单使用目的值和当前差值，然后在转换到范围0-2*PI
         * @param cur in 当前弧度值
         * @param dst in 目的弧度值
        */
        static float getRadVel(float cur,float dst);

        /**
         * @brief 依据当前locz目的locz计算俯仰值
         * 只简单使用目的值和当前差值除以5
         * @param curZ in 当前z值
         * @param dstZ in 目的z值
        */
        static float getPitchAngFrmZ(float curZ,float dstZ);

        /**
         * @brief 依据当前loc和目的loc计算yaw，0----2*PI 值
         * @param x1 in 起始点北向值
         * @param y1 in 起始点东向值
         * @param x2 in 目的点北向值
         * @param y2 in 目的点东向值
        */
        static float getYawFrmLoc(float x1,float y1,float x2,float y2);
        /**
         * @brief 依据当前loc和目的loc计算yaw，-PI ---- PI
         * 先计算0----2*PI 值，再转换为 -PI ---- PI
         * @param x1 in 起始点北向值
         * @param y1 in 起始点东向值
         * @param x2 in 目的点北向值
         * @param y2 in 目的点东向值
        */
        static float getYawN2PFrmLoc(float x1,float y1,float x2,float y2);
        
        /**
         * @brief 依据当前loc和目的loc计算两点直接的夹角，弧度
         * @param x1 in 起始点北向值
         * @param y1 in 起始点东向值
         * @param x2 in 目的点北向值
         * @param y2 in 目的点东向值
        */
        static float getRadFrmLoc(float x1,float y1,float x2,float y2);

        /**
         * @brief 计算某相同纬度上，两点的locy差值
         * @param lat in 纬度值
         * @param diffLon in 相差的经度值
        */
        inline static double getDiffY(double lat, double diffLon){
            return cos(ang2rad(lat)) * ms_radius * ang2rad(diffLon);

        }

        /**
         * @brief 依据当前2点的gps值，单位deg，计算loc距离
         * @param lat1 in 第一点纬度
         * @param lon1 in 第一点经度
         * @param lat2 in 第二点纬度
         * @param lon2 in 第二点经度
        */
        static double getHorDisFrmAng(double lat1,double lon1,double lat2, double lon2);

        /**
         * @brief 依据当前2点的gps值，单位弧度，计算loc距离
         * @param lat1 in 第一点纬度
         * @param lon1 in 第一点经度
         * @param lat2 in 第二点纬度
         * @param lon2 in 第二点经度
        */
        inline static double getHorDisFrmRad(double lat1, double lon1, double lat2, double lon2){
            return ms_radius * sqrt(2 * (1 - cos(lat1) * cos(lat2) * cos(lon2 - lon1) - sin(lat1) * sin(lat2)));
        }

        /**
         * @brief 依据2点loc的水平值，计算2点之间的水平距离，单位米
         * @param x1 in 第一点北向值
         * @param y1 in 第一点东向值
         * @param x2 in 第二点北向值
         * @param y2 in 第二点东向值
        */
        inline static float getDisFrmLoc(float x1, float y1, float x2, float y2){
            return (float)sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        }
      
        /**
         * @brief 依据2点loc的三维值，计算2点之间的三维距离，单位米
         * @param x1 in 第一点北向值
         * @param y1 in 第一点东向值
         * @param z1 in 第一点向地值
         * @param x2 in 第二点北向值
         * @param y2 in 第二点东向值
         * @param z2 in 第二点向地值
        */
        inline static float getDisFrmLoc(float x1,float y1,float z1,float x2,float y2,float z2){
            return (float)sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)+(z1-z2)*(z1-z2));
        }
        
        /**
         * @brief 归一化计算，给定值在给定范围的比例值
         * @param val in 输入值
         * @param max in 范围极大值
         * @param min in 范围极小值
        */
        inline static float oneNormalized(float val,float max = 100, float min = 0){
            return (val-min) / (max-min);
        }

        /**
         * @brief 米每秒转换为千米没小时
         * @param val in 输入值
        */
        inline static float ms2kmh(float val){
            return (float)(val * 3.6);
        }
        
        /**
         * @brief 千米没小时转换为米每秒
         * @param val in 输入值
        */
        inline static float kmh2ms(float val){
            return (float)(val * 5.0 / 18.0);
        }
        /**
         * @brief 求（x0,y0)点 到（x1,y1)和(x2,y2)2点构成直线的最短距离
         * @param x1 第一点北向值
         * @param y1 第一点东向值
         * @param x2 第二点北向值
         * @param y2 第二点东向值
         * @param x0 计算点北向值
         * @param y0 计算点东向值
         */ 
        static float getMinDis(float &x1,float &y1,float &x2,float &y2,float x0,float y0);

        /**
         * @brief 计算当前点选择一定角度后的北向值
         * @param x in 当前点北向值
         * @param y in 当前点东向值
         * @param ang in 旋转角度（弧度）
        */
        inline static float getRotax(float &x,float &y,float &ang) {
            return (float)(cos(ang) * x - sin(ang) * y);
        }

        /**
         * @brief 计算当前点选择一定角度后的北向值
         * @param x in 当前点北向值
         * @param y in 当前点东向值
         * @param ang in 旋转角度（弧度）
        */
        inline static float getRotax(float x, float y, float ang){
            return (float)(cos(ang) * x - sin(ang) * y);
        }

        /**
         * @brief 计算当前点选择一定角度后的东向值
         * @param x in 当前点北向值
         * @param y in 当前点东向值
         * @param ang in 旋转角度（弧度）
        */
        inline static float getRotay(float &x, float &y, float &ang){
            return (float)(cos(ang) * y + sin(ang) * x);
        }

        /**
         * @brief 计算当前点选择一定角度后的东向值
         * @param x in 当前点北向值
         * @param y in 当前点东向值
         * @param ang in 旋转角度（弧度）
        */
        inline static float getRotay(float x, float y, float ang){
            return (float)(cos(ang) * y + sin(ang) * x);
        }

        /**
         * @brief 计算当前点选择一定角度后的水平loc值
         * @param x in 当前点北向值
         * @param y in 当前点东向值
         * @param ang in 旋转角度（弧度）
         * @param rsltx out 旋转后的北向值
         * @param rslty out 旋转后的东向值
        */
        inline static void getRotaxy(float x,float y,float ang,float &rsltx,float &rslty){
            rsltx = (float)(cos(ang) * x - sin(ang) * y);
            rslty = (float)(cos(ang) * y + sin(ang) * x);
        }

        /**
         * @brief 计算点从一个范围到另一个范围的对应值
         * @param max1 in 输入范围极大值
         * @param min1 in 输入范围极小值
         * @param val1 in 在输入范围的取的待计算值
         * @param max2 in 输出范围极大值
         * @param min2 in 输出范围极小值
        */
        static float val1toval2(float max1,float min1, float val1,float max2, float min2);

        /**
         * @brief 获取当前值乘数因子后的值，并设置了值上限
         * @param val in 待计算值
         * @param op in 乘数因子
         * @param maxval in 最大限定值
        */
        inline static float multiOp(float val,float op,float maxVal){
            if (val * op > maxVal)
                return maxVal;
            return val * op;
        }
        /**
         * @brief 计算离点p1 p2组成的线段,垂直距离为dis的两点 
        */
        static void getVerPtFrmLine(const Point & pt1,const Point & pt2,Point &out1,Point &out2,
        float dis);

        /**
         * @brief 找出 p1和p2中离src最近的点
        */
        inline static Point getNearPt(const Point & pt1,const Point & pt2,const Point & src){
            if(bg::distance(pt1,src) < bg::distance(pt2,src))
                return pt1;
            return pt2;
        }

        /**
         * @brief 找出 p1和p2中离src最近的点
        */
        static Point getNearPt(const float & x1,const float & y1,const float & x2,const float & y2,
        const float & xsrc,const float & ysrc,float  dis);

        /**
         * @brief 将系列gps值(水平向)转换为loc值
         * @param home in home点
         * @param line inout 存放gps值及转换后的值
        */
        static void gps2loc(const geometry_msgs::msg::Point & home,
        std::vector<geometry_msgs::msg::Point32> &line);

        /**
         * @brief 将单点gps值(水平向)转换为loc值
         * @param home in home点
         * @param pt inout 存放gps值及转换后的值
        */
        static void gps2loc(const geometry_msgs::msg::Point & home,geometry_msgs::msg::Point32 & pt);

        template<typename typ>
        static void checkZValid(typ &val){
            if(val>0){
                val *= -1;
            }
            if(abs(val )<1e-6){
                val = EDefVal::DefZ;
            }
        }

        template<typename typ>
        static void getPositiveVal(typ &val){
            if(val < 0){
                val *= -1;
            }
        }

        static double ms_radius;// = 6378140.0;//地球半径米
        static double ms_zero;// = 1e-6;//0

    };
    

    } // namespace algorithm
    
} // namespace zyzn

#endif