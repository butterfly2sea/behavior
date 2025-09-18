#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>

#include <cmath>
#include <vector>

/**
 * 地理坐标结构
 */
struct GeographicCoordinate {
  double latitude;   // 纬度 (度)
  double longitude;  // 经度 (度)
  double altitude;   // 高度 (米，相对于WGS84椭球)

  GeographicCoordinate() : latitude(0.0), longitude(0.0), altitude(0.0) {}
  GeographicCoordinate(double lat, double lon, double alt)
      : latitude(lat), longitude(lon), altitude(alt) {}
};

/**
 * 本地坐标结构 (NED - North East Down)
 */
struct LocalCoordinate {
  double north;  // 北向 (米)
  double east;   // 东向 (米)  
  double down;   // 下向 (米)

  LocalCoordinate() : north(0.0), east(0.0), down(0.0) {}
  LocalCoordinate(double n, double e, double d)
      : north(n), east(e), down(d) {}
};

/**
 * ECEF坐标结构 (Earth-Centered, Earth-Fixed)
 */
struct ECEFCoordinate {
  double x;  // X轴坐标 (米)
  double y;  // Y轴坐标 (米)
  double z;  // Z轴坐标 (米)

  ECEFCoordinate() : x(0.0), y(0.0), z(0.0) {}
  ECEFCoordinate(double x_val, double y_val, double z_val)
      : x(x_val), y(y_val), z(z_val) {}
};

/**
 * 坐标转换工具类 - 提供各种坐标系之间的转换
 */
class CoordinateConverter {
 private:
  // WGS84椭球参数
  static constexpr double WGS84_A = 6378137.0;           // 长半轴 (米)
  static constexpr double WGS84_F = 1.0 / 298.257223563; // 扁率
  static constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F; // 偏心率平方

  // 参考点 (本地坐标系原点)
  static GeographicCoordinate reference_point_;
  static bool reference_set_;

 public:
  // ============= 参考点管理 =============

  /**
   * 设置本地坐标系参考点
   * @param lat 纬度 (度)
   * @param lon 经度 (度)
   * @param alt 高度 (米)
   */
  static void setReferencePoint(double lat, double lon, double alt);

  /**
   * 设置本地坐标系参考点
   * @param ref 参考点坐标
   */
  static void setReferencePoint(const GeographicCoordinate& ref);

  /**
   * 获取当前参考点
   * @return 参考点坐标
   */
  static GeographicCoordinate getReferencePoint();

  /**
   * 检查参考点是否已设置
   * @return 是否已设置
   */
  static bool isReferenceSet();

  /**
   * 重置参考点
   */
  static void resetReference();

  // ============= 基础坐标转换 =============

  /**
   * 地理坐标转ECEF坐标
   * @param geo 地理坐标
   * @return ECEF坐标
   */
  static ECEFCoordinate geographicToECEF(const GeographicCoordinate& geo);

  /**
   * ECEF坐标转地理坐标
   * @param ecef ECEF坐标
   * @return 地理坐标
   */
  static GeographicCoordinate ecefToGeographic(const ECEFCoordinate& ecef);

  /**
   * 地理坐标转本地NED坐标
   * @param geo 地理坐标
   * @return 本地NED坐标 (需要先设置参考点)
   */
  static LocalCoordinate geographicToLocal(const GeographicCoordinate& geo);

  /**
   * 本地NED坐标转地理坐标
   * @param local 本地NED坐标
   * @return 地理坐标 (需要先设置参考点)
   */
  static GeographicCoordinate localToGeographic(const LocalCoordinate& local);

  // ============= ROS消息转换 =============

  /**
   * geometry_msgs::Point转本地坐标
   * @param point ROS点消息
   * @return 本地坐标
   */
  static LocalCoordinate pointToLocal(const geometry_msgs::msg::Point& point);

  /**
   * 本地坐标转geometry_msgs::Point
   * @param local 本地坐标
   * @return ROS点消息
   */
  static geometry_msgs::msg::Point localToPoint(const LocalCoordinate& local);

  /**
   * geometry_msgs::Point转地理坐标
   * @param point ROS点消息
   * @return 地理坐标
   */
  static GeographicCoordinate pointToGeographic(const geometry_msgs::msg::Point& point);

  /**
   * 地理坐标转geometry_msgs::Point
   * @param geo 地理坐标
   * @return ROS点消息
   */
  static geometry_msgs::msg::Point geographicToPoint(const GeographicCoordinate& geo);

  /**
   * SimpleVehicle消息转地理坐标
   * @param vehicle 飞行器状态消息
   * @return 地理坐标
   */
  static GeographicCoordinate simpleVehicleToGeographic(const custom_msgs::msg::SimpleVehicle& vehicle);

  /**
   * SimpleVehicle消息转本地坐标
   * @param vehicle 飞行器状态消息
   * @return 本地坐标
   */
  static LocalCoordinate simpleVehicleToLocal(const custom_msgs::msg::SimpleVehicle& vehicle);

  // ============= 批量转换 =============

  /**
   * 转换航点列表到本地坐标
   * @param waypoints 地理坐标航点列表
   * @return 本地坐标航点列表
   */
  static geometry_msgs::msg::Polygon geographicWaypointsToLocal(
      const std::vector<GeographicCoordinate>& waypoints);

  /**
   * 转换本地航点列表到地理坐标
   * @param polygon 本地坐标航点多边形
   * @return 地理坐标航点列表
   */
  static std::vector<GeographicCoordinate> localWaypointsToGeographic(
      const geometry_msgs::msg::Polygon& polygon);

  // ============= 距离和角度计算 =============

  /**
   * 计算两个地理坐标间的距离 (Haversine公式)
   * @param geo1 坐标1
   * @param geo2 坐标2
   * @return 距离 (米)
   */
  static double calculateDistance(const GeographicCoordinate& geo1,
                                  const GeographicCoordinate& geo2);

  /**
   * 计算两个本地坐标间的距离
   * @param local1 坐标1
   * @param local2 坐标2
   * @return 距离 (米)
   */
  static double calculateDistance(const LocalCoordinate& local1,
                                  const LocalCoordinate& local2);

  /**
   * 计算两个地理坐标间的方位角
   * @param from 起始坐标
   * @param to 目标坐标
   * @return 方位角 (度, 0-360, 0为正北)
   */
  static double calculateBearing(const GeographicCoordinate& from,
                                 const GeographicCoordinate& to);

  /**
   * 计算两个本地坐标间的方位角
   * @param from 起始坐标
   * @param to 目标坐标
   * @return 方位角 (度, 0-360, 0为正北)
   */
  static double calculateBearing(const LocalCoordinate& from,
                                 const LocalCoordinate& to);

  // ============= 实用工具方法 =============

  /**
   * 计算地理坐标在给定距离和方位角处的新坐标
   * @param origin 起始坐标
   * @param distance 距离 (米)
   * @param bearing 方位角 (度)
   * @return 新坐标
   */
  static GeographicCoordinate offsetGeographic(const GeographicCoordinate& origin,
                                               double distance,
                                               double bearing);

  /**
   * 计算本地坐标的偏移
   * @param origin 起始坐标
   * @param north_offset 北向偏移 (米)
   * @param east_offset 东向偏移 (米)
   * @param down_offset 下向偏移 (米)
   * @return 新坐标
   */
  static LocalCoordinate offsetLocal(const LocalCoordinate& origin,
                                     double north_offset,
                                     double east_offset,
                                     double down_offset);

  /**
   * 归一化角度到0-360度范围
   * @param angle 角度 (度)
   * @return 归一化后的角度
   */
  static double normalizeAngle(double angle);

  /**
   * 将角度从度转换为弧度
   * @param degrees 角度 (度)
   * @return 角度 (弧度)
   */
  static double degreesToRadians(double degrees);

  /**
   * 将角度从弧度转换为度
   * @param radians 角度 (弧度)
   * @return 角度 (度)
   */
  static double radiansToDegrees(double radians);

  // ============= 坐标系验证 =============

  /**
   * 验证地理坐标是否有效
   * @param geo 地理坐标
   * @return 是否有效
   */
  static bool isValidGeographic(const GeographicCoordinate& geo);

  /**
   * 验证本地坐标是否合理 (相对于参考点)
   * @param local 本地坐标
   * @param max_distance 最大允许距离 (米)
   * @return 是否合理
   */
  static bool isValidLocal(const LocalCoordinate& local, double max_distance = 50000.0);

  /**
   * 检查两个坐标是否接近
   * @param geo1 坐标1
   * @param geo2 坐标2
   * @param tolerance 容差 (米)
   * @return 是否接近
   */
  static bool areCoordinatesClose(const GeographicCoordinate& geo1,
                                  const GeographicCoordinate& geo2,
                                  double tolerance = 1.0);

  // ============= 调试和信息 =============

  /**
   * 将地理坐标转换为字符串
   * @param geo 地理坐标
   * @return 字符串表示
   */
  static std::string geographicToString(const GeographicCoordinate& geo);

  /**
   * 将本地坐标转换为字符串
   * @param local 本地坐标
   * @return 字符串表示
   */
  static std::string localToString(const LocalCoordinate& local);

  /**
   * 获取坐标转换信息
   * @return 转换器状态信息
   */
  static std::string getConverterInfo();

 private:
  // ============= 内部辅助方法 =============

  /**
   * 检查参考点是否已设置 (抛出异常如果未设置)
   */
  static void ensureReferenceSet();

  /**
   * 计算卯酉圈曲率半径
   * @param lat 纬度 (弧度)
   * @return 曲率半径
   */
  static double calculatePrimeVerticalRadius(double lat);

  /**
   * 计算子午圈曲率半径
   * @param lat 纬度 (弧度)
   * @return 曲率半径
   */
  static double calculateMeridionalRadius(double lat);
};