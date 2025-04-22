#include "../../include/utility/Utility.hpp"

namespace zyzn{
    namespace algorithm{
        double CUtility::ms_zero = 1e-6;
        double CUtility::ms_radius = 6378140.0;
        float CUtility::getRadVel(float cur,float dst){
            float vel = (dst - cur);
            if (vel > M_PI)
            {
                vel = vel- (float)M_PI * 2;
            }
            if(vel < -1 * M_PI)
            {
                vel = (float)M_PI * 2 + vel;
            }
            return vel;
        }

        float CUtility::getPitchAngFrmZ(float curZ,float dstZ){
            float dst = (float)((dstZ - curZ)/5.0);
            //if (dst > ParamMgr.ins().ctlZone.climM_PItch)
            //    dst = ParamMgr.ins().ctlZone.climM_PItch;
            //else if (dst < -1 * ParamMgr.ins().ctlZone.slipM_PItch)
            //    dst = -1 * ParamMgr.ins().ctlZone.slipM_PItch;
            return dst;
        }

        float CUtility::getYawFrmLoc(float x1,float y1,float x2,float y2){
            float ang = getRadFrmLoc(x1, y1, x2, y2);
            ang -= (float)M_PI / 2;
            if (ang < 0)
                ang += (float)M_PI * 2;
            return (float)(M_PI * 2 - ang);
        }

        float CUtility::getYawN2PFrmLoc(float x1,float y1,float x2,float y2){
            float yaw =getYawFrmLoc(x1,y1,x2,y2);
            if (yaw > M_PI)
            {
                yaw -= (float)M_PI * 2;
            }
            return yaw;
        }

        float CUtility::getRadFrmLoc(float x1,float y1,float x2,float y2){
            if (abs(y2 - y1) < ms_zero){
                
                if (x2 > x1)
                    return (float)M_PI/2;
                else
                    return (float)(3*M_PI/2);
            }
            if(abs(x2-x1) < ms_zero){
                if (y2 > y1)
                    return 0;
                else
                    return (float)M_PI;
            }
                
            float ang = (float)atan((x2 - x1) / (y2 - y1));
            if (x2>x1 && y2>y1){
                return ang;
            }
            else if(x2>x1 && y1 > y2){
                return (float)M_PI+ang;
            }
            else if(x1>x2 && y1>y2){
                return (float)(M_PI  + ang);
            }
            else{
                return (float)(M_PI * 2 - ang);
            }
        
        }

        double CUtility::getHorDisFrmAng(double lat1,double lon1,double lat2, double lon2){
            lat1 = ang2rad(lat1);
            lat2 = ang2rad(lat2);
            lon1 = ang2rad(lon1);
            lon2 = ang2rad(lon2);
            return getHorDisFrmRad(lat1, lon1, lat2, lon2);
        }

        float CUtility::getMinDis(float &x1,float &y1,float &x2,float &y2,float x0,float y0){
            float rad2 = getRadFrmLoc(x1, y1, x2, y2);
            float rad0 = getRadFrmLoc(x1, y1, x0, y0);
            return (float)(abs(getDisFrmLoc(x1,y1,x0,y0)*sin(rad0-rad2)));
        }

        float CUtility::val1toval2(float max1,float min1, float val1,float max2, float min2){
            if (val1 <= min1)
                return min2;
            if (val1 >= max1)
                return max2;
            return min2 + (max2 - min2) * val1 / (max1 - min1);
        }
        
   
        void CUtility::gps2loc(const geometry_msgs::msg::Point & home,
        std::vector<geometry_msgs::msg::Point32> &line){
            struct map_projection_reference_s target_ref{};
            map_projection_init(&target_ref, home.x/*lat*/, home.y/*lon*/);
            for(int i=0;i<line.size();++i){               
                map_projection_project(&target_ref, line[i].x, line[i].y, &line[i].x, &line[i].y);               
    
            }
            
        }

        void CUtility::gps2loc(const geometry_msgs::msg::Point & home,geometry_msgs::msg::Point32 & pt){
            struct map_projection_reference_s target_ref{};
            map_projection_init(&target_ref, home.x/*lat*/, home.y/*lon*/);
            map_projection_project(&target_ref, pt.x, pt.y, &pt.x, &pt.y);   

        }



    }
}