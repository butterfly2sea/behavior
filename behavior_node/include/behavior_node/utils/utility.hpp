#pragma once

#include <string>
#include <vector>
#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include "behavior_node/data/base_enum.hpp"

namespace utility {

static const double earth_radius = 6378140.0; // 地球半径米


// 地图投影参考结构体
struct MapProjectionReference {
  double lat_rad;
  double lon_rad;
  double sin_lat;
  double cos_lat;
  bool initialized;

  MapProjectionReference() : lat_rad(0), lon_rad(0),
                             sin_lat(0), cos_lat(0), initialized(false) {}
};

inline static double ang2rad(double ang) { return M_PI * ang / 180; }
inline static double rad2ang(double rad) { return 180 * rad / M_PI; }

/**
* @brief 依据纬度差计算locx差值，只简单使用地球半径和弧度角度值计算
* @param diffLat 纬度差值，度数
* @return locx的差值
*/
inline static double getDiffX(double diffLat) { return ang2rad(diffLat) * earth_radius; }

/**
 * @brief 依据当前弧度和目的弧度计算速度值（弧度）
 * 只简单使用目的值和当前差值，然后在转换到范围0-2*PI
 * @param cur 当前弧度
 * @param dst 目的弧度
 * @return 两点之间的距离，单位为米
 */
inline static float getRadVel(float cur, float dst) {
  if (float dif = dst - cur; dif > M_PI) { return dif - 2 * M_PI; }
  else if (dif < -M_PI) { return dif + 2 * M_PI; } else { return dif; }
}

/**
 * @brief 依据当前current_z目的destiny_z计算俯仰值
 * 只简单使用目的值和当前差值除以5
 * @param current_z 当前俯仰值
 * @param destiny_z 目的俯仰值
 * @return 俯仰值差值
 */
inline static float getPitchAngleFromZ(float current_z, float destiny_z) {
  return (float) ((current_z - destiny_z) / 5.0);
}

/**
 * @brief 依据当前loc和目的loc计算yaw，-PI ---- PI 值域
 * @param x1 in 起始点北向值
 * @param y1 in 起始点东向值
 * @param x2 in 目的点北向值
 * @param y2 in 目的点东向值
 * @return 偏航角
 */
inline static float getYawToDest(float x1, float y1, float x2, float y2) { return atan2(y2 - y1, x2 - x1); }

/**
 * @brief 计算某相同纬度上，两点的locy差值
 * @param lat in 纬度值
 * @param diffLon in 相差的经度值
 * @return 经度上的距离
*/
inline static double getDiffY(double lat, double diffLon) {
  return cos(ang2rad(lat)) * earth_radius * ang2rad(diffLon);
}

/**
 *  @brief 计算两点之间的距离，单位为米
 *  @param lat1 in 起始点纬度值
 *  @param lon1 in 起始点经度值
 *  @param lat2 in 目的点纬度值
 *  @param lon2 in 目的点经度值
 *  @return 两点之间的距离，单位为米
 */
inline static double getHorDisFrmRad(double lat1, double lon1, double lat2, double lon2) {
  return earth_radius * acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon2 - lon1));
}

/**
 * @brief 依据当前2点的gps值，单位deg，计算loc距离
 * @param lat1 in 第一点纬度
 * @param lon1 in 第一点经度
 * @param lat2 in 第二点纬度
 * @param lon2 in 第二点经度
*/
inline static double getHorDisFrmAng(double lat1, double lon1, double lat2, double lon2) {
  return getHorDisFrmRad(ang2rad(lat1), ang2rad(lon1), ang2rad(lat2), ang2rad(lon2));
}

/**
 * @brief 依据2点loc的水平值，计算2点之间的水平距离，单位米
 * @param x1 in 第一点北向值
 * @param y1 in 第一点东向值
 * @param x2 in 第二点北向值
 * @param y2 in 第二点东向值
 * @return 两点之间的水平距离，单位米
*/
inline static float getDisFrmLoc(float x1, float y1, float x2, float y2) {
  return (float) sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

/**
 * @brief 依据2点loc的三维值，计算2点之间的三维距离，单位米
 * @param x1 in 第一点北向值
 * @param y1 in 第一点东向值
 * @param z1 in 第一点向地值
 * @param x2 in 第二点北向值
 * @param y2 in 第二点东向值
 * @param z2 in 第二点向地值
 * @return 两点之间的三维距离，单位米
*/
inline static float getDisFrmLoc(float x1, float y1, float z1, float x2, float y2, float z2) {
  return (float) sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
}

/**
 * @brief 归一化计算，给定值在给定范围的比例值
 * @param val in 输入值
 * @param max in 范围极大值
 * @param min in 范围极小值
 * @return 归一化值
*/
inline static float oneNormalized(float val, float max = 100, float min = 0) {
  return (val - min) / (max - min);
}

/**
 * @brief 米每秒转换为千米没小时
 * @param val in 输入值
 * @return 千米每小时
*/
inline static float mps2kmph(float val) {
  return (float) (val * 3.6);
}
/**
 * @brief 千米每小时转换为米每秒
 * @param val in 输入值
 * @return 米每秒
*/
inline static float kmph2mps(float val) {
  return (float) (val / 3.6);
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
inline static float getMinDis(float &x1, float &y1, float &x2, float &y2, float x0, float y0) {
  return std::abs(getDisFrmLoc(x1, y1, x0, y0)) * sin(getYawToDest(x1, y1, x2, y2) - getYawToDest(x0, y0, x2, y2));
}

/**
 * 初始化地图投影
 * @param ref 投影参考结构体指针
 * @param lat_0 参考点纬度(度)
 * @param lon_0 参考点经度(度)
 * @return 0表示成功，-1表示失败
 */
int map_projection_init(MapProjectionReference *ref, double lat_0, double lon_0) {
  if (!ref) {
    return -1;
  }

  // 将度转换为弧度
  ref->lat_rad = ang2rad(lat_0);
  ref->lon_rad = ang2rad(lon_0);

  // 预计算三角函数值
  ref->sin_lat = std::sin(ref->lat_rad);
  ref->cos_lat = std::cos(ref->lat_rad);
  ref->initialized = true;

  return 0;
}

/**
 * 将地理坐标投影到平面坐标
 * @param ref 投影参考结构体指针
 * @param lat 纬度(度)
 * @param lon 经度(度)
 * @param x 输出的x坐标(米，指向北)
 * @param y 输出的y坐标(米，指向东)
 * @return 0表示成功，-1表示失败
 */
int map_projection_project(const MapProjectionReference *ref, double lat, double lon, float *x, float *y) {
  if (!ref || !ref->initialized || !x || !y) {
    return -1;
  }

  // 将度转换为弧度
  const double lat_rad = ang2rad(lat);
  const double lon_rad = ang2rad(lon);

  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);

  const double cos_d_lon = std::cos(lon_rad - ref->lon_rad);

  // 计算角c - 球面上两点间的角距离
  const double arg = std::clamp(ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon, -1.0, 1.0);
  const double c = std::acos(arg);

  // 计算比例系数k
  double k = 1.0;
  if (std::fabs(c) > 0) {
    k = (c / std::sin(c));
  }

  // 计算平面坐标
  *x = static_cast<float>(k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * earth_radius);
  *y = static_cast<float>(k * cos_lat * std::sin(lon_rad - ref->lon_rad) * earth_radius);

  return 0;
}

/**
 * @brief 将单点gps值(水平向)转换为loc值
 * @param home in home点
 * @param pt inout 存放gps值及转换后的值
*/
inline static void gps2loc(const geometry_msgs::msg::Point &home, geometry_msgs::msg::Point32 &pt) {
  struct MapProjectionReference target_ref{};
  map_projection_init(&target_ref, home.x/*lat*/, home.y/*lon*/);
  map_projection_project(&target_ref, pt.x, pt.y, &pt.x, &pt.y);
}

/**
 * @brief 将多个点gps值(水平向)转换为loc值
 * @param home in home点
 * @param line inout 存放gps值及转换后的值
*/
inline static void gps2loc(const geometry_msgs::msg::Point &home, std::vector<geometry_msgs::msg::Point32> &line) {
  struct MapProjectionReference target_ref{};
  map_projection_init(&target_ref, home.x/*lat*/, home.y/*lon*/);
  for (auto &i : line) {
    map_projection_project(&target_ref, i.x, i.y, &i.x, &i.y);
  }
}

template<typename typ>
static void checkZValid(typ &val) {
  if (val > 0) {
    val *= -1;
  }
  if (abs(val) < 1e-6) {
    val = -50;
  }
}

}  // namespace utility
