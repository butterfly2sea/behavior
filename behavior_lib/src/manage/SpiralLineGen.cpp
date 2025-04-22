#include <math.h>
#include <iostream>
#include "../../include/manage/SpiralLineGen.hpp"
#include "../../include/info/Param.hpp"

namespace zyzn{
    namespace manage{
        CSpiralLineGen::CSpiralLineGen(){

        }

        CSpiralLineGen::~CSpiralLineGen(){
        }

        void CSpiralLineGen::genline(const set::CSetDstPt::SAutoDstPt & dstPt,custom_msgs::msg::TaskStage::SharedPtr stage){
            if(dstPt.intval <= 0.000001)
            {
                return;
            }
            float R = dstPt.rdis;
            //std::cout<<"radius:"<<R<<" intval:"<<dstPt.intval<<std::endl;
            if(R < dstPt.intval)
                R = dstPt.intval;
            
           
            float circles = R/dstPt.intval;
            int totalNum = getTotalPts(R,dstPt.intval);
            if(totalNum <= 0){
                return;
            }
            stage.get()->line.points.reserve(totalNum);
            stage.get()->line.points.clear();
            //std::cout<<"circles:"<<circles<<"totalNum:"<<totalNum<<std::endl;
            
            for (float i = 0; i < circles; i++)
            {
                /* code */
                float scale = 1;
                float maxR = (i+1)*dstPt.intval;
                if(maxR > R){
                    maxR = R;
                    scale = circles - i;
                }
                
                int nums = getPtNums(maxR,dstPt.intval,scale);
                //std::cout<<"i:"<<i<<"nums:"<<nums<<"maxR:"<<maxR<<"scale:"<<scale<<std::endl;
                geometry_msgs::msg::Point32 pt;
                if(abs(dstPt.alt) < 0.0001)
                    pt.z = -1;
                else 
                    pt.z = dstPt.alt;
                for(int j = 0;j < nums;j++){
                    if(stage.get()->line.points.size() < 1000){
                        float r = i*dstPt.intval + scale*j*dstPt.intval/nums;
                    
                        pt.x = r*cos(2*M_PI*j/nums)+dstPt.dstPt.x;
                        pt.y = r*sin(2*M_PI*j/nums)+dstPt.dstPt.y;
                        
                        stage.get()->line.points.push_back(pt);
                    }
                }
            }
                
            
        }

        int CSpiralLineGen::getTotalPts(float R,float intv){
            // TODO
               float circles = R/intv;
               int nums = 0;
                
                for (float i = 0; i < circles; i++)
                {
                    /* code */
                    float scale = 1;
                    float maxR = (i+1)*intv;
                    if(maxR > R){
                        maxR = R;
                        scale = circles - i;
                    }
                    
                    nums += getPtNums(maxR,intv,scale);
                 
                }
            return nums;
        }
        int CSpiralLineGen::getPtNums(float r,float intv,float scale){
            // TODO
            int nums = scale*2*M_PI*r/intv;
            if(nums > MaxPtsPerCircle)
                nums = MaxPtsPerCircle;
            return nums;
        }
    }
}