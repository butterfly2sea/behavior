#ifndef ZYZN_MANAGE_SPIRALLINEGEN_HPP
#define ZYZN_MANAGE_SPIRALLINEGEN_HPP
#include <boost/signals2/signal.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include "../set/SetDstPt.hpp"

namespace zyzn{
    namespace manage{
        /**
         * @brief 螺旋线航线生成
         * @details 螺旋线航线生成，用于国科大详查点航线生成，以点为中心，最大搜索半径、搜索间隔为参数生成螺旋航线
         * @author zyzn
        */
        class CSpiralLineGen
        {

        public:
            enum {
                MaxPtsPerCircle=300//每圈最多点数
            };

            typedef void (f_genline_t) (const set::CSetDstPt::SAutoDstPt &,custom_msgs::msg::TaskStage::SharedPtr);
            typedef boost::signals2::signal<f_genline_t> sig_genline_t;
            sig_genline_t sig_genline;

            CSpiralLineGen(/* args */);
            ~CSpiralLineGen();

            /**
             * @brief 生成螺旋航线
             * @details 生成螺旋航线，以点为中心，最大搜索半径、搜索间隔为参数生成螺旋航线
             * @param dstPt in 详查目的点
             * @param stage in out 任务阶段，会把生成的航线更新到此任务中
            */
            void genline(const set::CSetDstPt::SAutoDstPt & dstPt,custom_msgs::msg::TaskStage::SharedPtr stage);
            private:

            /**
             * @brief 获取总航点数
             * @details 最大搜索半径、搜索间隔为参数计算可生成螺旋航线的航点数，累计每圈的航点数
             * @param R in 半径
             * @param intv in 扫描间隔
             * @return int 总航点数
            */
            int getTotalPts(float R,float intv);

            /**
             * @brief 获取每圈的航点数
             * @details 搜索半径、搜索间隔为参数计算每圈的航点数
             * @param R in 半径
             * @param intv in 扫描间隔
             * @param scale in 缩放比例
             * @return int 每圈的航点数
            */
            int getPtNums(float r,float intv,float scale=1.0);


        };

    }

}





#endif