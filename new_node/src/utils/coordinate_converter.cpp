#include "behavior_node/utils/coordinate_converter.hpp"
#include <log/Logger.hpp>
#include <cmath>

namespace behavior_node {

// 地球参数常量
constexpr double EARTH_RADIUS_M = 6378137.0;       // WGS84椭球长半轴
constexpr double EARTH_FLATTENING = 1.0 / 298.257223563;  // WGS84椭球扁率
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

CoordinateConverter::CoordinateConverter() {
  txtLog().debug("UTIL CoordinateConverter created");

  // 设置默认参考点（如果需要的话）
  reference_point_.latitude = 0.0;
  reference_point_.longitude = 0.0;
  reference_point_.altitude = 0.0;
  reference_set_ = false;
}

CoordinateConverter::~CoordinateConverter() = default;

// ================================ 参考点设置 ================================

void CoordinateConverter::setReferencePoint(double lat, double lon, double alt) {
  reference_point_.latitude = lat;
  reference_point_.longitude = lon;
  reference_point_.altitude = alt;
  reference_set_ = true;

  // 预计算参考点的一些值以优化后续转换
  precomputeReferenceValues();

  txtLog().info("UTIL CoordinateConverter reference point set: lat=%.8f, lon=%.8f, alt=%.2f",
                lat, lon, alt);
}

void CoordinateConverter::setReferencePoint(const GeodeticPosition& ref_point) {
  setReferencePoint(ref_point.latitude, ref_point.longitude, ref_point.altitude);
}

GeodeticPosition CoordinateConverter::getReferencePoint() const {
  return reference_point_;
}

bool CoordinateConverter::isReferenceSet() const {
  return reference_set_;
}

void CoordinateConverter::precomputeReferenceValues() {
  if (!reference_set_) return;

  double lat_rad = reference_point_.latitude * DEG_TO_RAD;

  // 计算参考点的曲率半径
  double sin_lat = std::sin(lat_rad);
  double e2 = 2.0 * EARTH_FLATTENING - EARTH_FLATTENING * EARTH_FLATTENING;
  double N = EARTH_RADIUS_M / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

  ref_cos_lat_ = std::cos(lat_rad);
  ref_sin_lat_ = sin_lat;
  ref_radius_n_ = N;
  ref_radius_m_ = N * (1.0 - e2) / (1.0 - e2 * sin_lat * sin_lat);
}

// ================================ 大地坐标与本地坐标转换 ================================

LocalPosition CoordinateConverter::geodeticToLocal(const GeodeticPosition& geodetic) const {
  if (!reference_set_) {
    txtLog().error("UTIL CoordinateConverter reference point not set for geodetic to local conversion");
    return LocalPosition{0.0f, 0.0f, 0.0f};
  }

  try {
    double dlat = (geodetic.latitude - reference_point_.latitude) * DEG_TO_RAD;
    double dlon = (geodetic.longitude - reference_point_.longitude) * DEG_TO_RAD;
    double dalt = geodetic.altitude - reference_point_.altitude;

    // 使用切平面投影近似
    LocalPosition local;
    local.x = static_cast<float>(dlat * ref_radius_m_);
    local.y = static_cast<float>(dlon * ref_radius_n_ * ref_cos_lat_);
    local.z = static_cast<float>(dalt);

    return local;

  } catch (const std::exception& e) {
    txtLog().error("UTIL CoordinateConverter geodetic to local conversion failed: %s", e.what());
    return LocalPosition{0.0f, 0.0f, 0.0f};
  }
}

GeodeticPosition CoordinateConverter::localToGeodetic(const LocalPosition& local) const {
  if (!reference_set_) {
    txtLog().error("UTIL CoordinateConverter reference point not set for local to geodetic conversion");
    return GeodeticPosition{0.0, 0.0, 0.0};
  }

  try {
    // 从本地坐标转换回大地坐标
    double dlat = static_cast<double>(local.x) / ref_radius_m_;
    double dlon = static_cast<double>(local.y) / (ref_radius_n_ * ref_cos_lat_);

    GeodeticPosition geodetic;
    geodetic.latitude = reference_point_.latitude + dlat * RAD_TO_DEG;
    geodetic.longitude = reference_point_.longitude + dlon * RAD_TO_DEG;
    geodetic.altitude = reference_point_.altitude + static_cast<double>(local.z);

    return geodetic;

  } catch (const std::exception& e) {
    txtLog().error("UTIL CoordinateConverter local to geodetic conversion failed: %s", e.what());
    return GeodeticPosition{0.0, 0.0, 0.0};
  }
}

// ================================ ECEF坐标系转换 ================================

ECEFPosition CoordinateConverter::geodeticToECEF(const GeodeticPosition& geodetic) const {
  try {
    double lat_rad = geodetic.latitude * DEG_TO_RAD;
    double lon_rad = geodetic.longitude * DEG_TO_RAD;
    double alt = geodetic.altitude;

    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double sin_lon = std::sin(lon_rad);
    double cos_lon = std::cos(lon_rad);

    // WGS84椭球参数
    double e2 = 2.0 * EARTH_FLATTENING - EARTH_FLATTENING * EARTH_FLATTENING;
    double N = EARTH_RADIUS_M / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

    ECEFPosition ecef;
    ecef.x = (N + alt) * cos_lat * cos_lon;
    ecef.y = (N + alt) * cos_lat * sin_lon;
    ecef.z = (N * (1.0 - e2) + alt) * sin_lat;

    return ecef;

  } catch (const std::exception& e) {
    txtLog().error("UTIL CoordinateConverter geodetic to ECEF conversion failed: %s", e.what());
    return ECEFPosition{0.0, 0.0, 0.0};
  }
}

GeodeticPosition CoordinateConverter::ecefToGeodetic(const ECEFPosition& ecef) const {
  try {
    double x = ecef.x;
    double y = ecef.y;
    double z = ecef.z;

    double p = std::sqrt(x * x + y * y);
    double lon = std::atan2(y, x);

    // 迭代计算纬度和高度
    double e2 = 2.0 * EARTH_FLATTENING - EARTH_FLATTENING * EARTH_FLATTENING;
    double lat = std::atan2(z, p * (1.0 - e2));
    double alt = 0.0;

    const int max_iterations = 10;
    const double tolerance = 1e-12;

    for (int i = 0; i < max_iterations; ++i) {
      double sin_lat = std::sin(lat);
      double N = EARTH_RADIUS_M / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

      double alt_new = p / std::cos(lat) - N;
      double lat_new = std::atan2(z, p * (1.0 - e2 * N / (N + alt_new)));

      if (std::abs(lat_new - lat) < tolerance && std::abs(alt_new - alt) < tolerance) {
        lat = lat_new;
        alt = alt_new;
        break;
      }

      lat = lat_new;
      alt = alt_new;
    }

    GeodeticPosition geodetic;
    geodetic.latitude = lat * RAD_TO_DEG;
    geodetic.longitude = lon * RAD_TO_DEG;
    geodetic.altitude = alt;

    return geodetic;

  } catch (const std::exception& e) {
    txtLog().error("UTIL CoordinateConverter ECEF to geodetic conversion failed: %s", e.what());
    return GeodeticPosition{0.0, 0.0, 0.0};
  }
}

// ================================ 距离和方位角计算 ================================

double CoordinateConverter::calculateDistance(const GeodeticPosition& pos1,
                                              const GeodeticPosition& pos2) const {
  try {
    // 使用Haversine公式计算大圆距离
    double lat1_rad = pos1.latitude * DEG_TO_RAD;
    double lat2_rad = pos2.latitude * DEG_TO_RAD;
    double dlat = (pos2.latitude - pos1.latitude) * DEG_TO_RAD;
    double dlon = (pos2.longitude - pos1.longitude) * DEG_TO_RAD;

    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
        std::cos(lat1_rad) * std::cos(lat2_rad) *
            std::sin(dlon / 2.0) * std::sin(dlon / 2.0);

    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double distance_2d = EARTH_RADIUS_M * c;

    // 考虑高度差
    double dalt = pos2.altitude - pos1.altitude;
    double distance_3d = std::sqrt(distance_2d * distance_2d + dalt * dalt);

    return distance_3d;

  } catch (const std::exception& e) {
    txtLog().error("UTIL CoordinateConverter distance calculation failed: %s", e.what());
    return 0.0;
  }
}

double CoordinateConverter::calculateBearing(const GeodeticPosition& from,
                                             const GeodeticPosition& to) const {
  try {
    double lat1_rad = from.latitude * DEG_TO_RAD;
    double lat2_rad = to.latitude * DEG_TO_RAD;
    double dlon_rad = (to.longitude - from.longitude) * DEG_TO_RAD;

    double y = std::sin(dlon_rad) * std::cos(lat2_rad);
    double x = std::cos(lat1_rad) * std::sin(lat2_rad) -
        std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon_rad);

    double bearing_rad = std::atan2(y, x);
    double bearing_deg = bearing_rad * RAD_TO_DEG;

    // 归一化到 [0, 360) 度
    if (bearing_deg < 0.0) {
      bearing_deg += 360.0;
    }

    return bearing_deg;

  } catch (const std::exception& e) {
    txtLog().error("UTIL CoordinateConverter bearing calculation failed: %s", e.what());
    return 0.0;
  }
}

GeodeticPosition CoordinateConverter::calculateDestination(const GeodeticPosition& start,
                                                           double distance,
                                                           double bearing) const {
  try {
    double lat1_rad = start.latitude * DEG_TO_RAD;
    double lon1_rad = start.longitude * DEG_TO_RAD;
    double bearing_rad = bearing * DEG_TO_RAD;
    double angular_distance = distance / EARTH_RADIUS_M;

    double lat2_rad = std::asin(std::sin(lat1_rad) * std::cos(angular_distance) +
        std::cos(lat1_rad) * std::sin(angular_distance) *
            std::cos(bearing_rad));

    double lon2_rad = lon1_rad + std::atan2(std::sin(bearing_rad) * std::sin(angular_distance) *
                                                std::cos(lat1_rad),
                                            std::cos(angular_distance) - std::sin(lat1_rad) *
                                                std::sin(lat2_rad));

    GeodeticPosition destination;
    destination.latitude = lat2_rad * RAD_TO_DEG;
    destination.longitude = lon2_rad * RAD_TO_DEG;
    destination.altitude = start.altitude;  // 保持相同高度

    return destination;

  } catch (const std::exception& e) {
    txtLog().error("UTIL CoordinateConverter destination calculation failed: %s", e.what());
    return start;
  }
}

// ================================ ROS消息转换 ================================

LocalPosition CoordinateConverter::pointToLocal(const geometry_msgs::msg::Point& point) const {
  LocalPosition local;
  local.x = static_cast<float>(point.x);
  local.y = static_cast<float>(point.y);
  local.z = static_cast<float>(point.z);
  return local;
}

geometry_msgs::msg::Point CoordinateConverter::localToPoint(const LocalPosition& local) const {
  geometry_msgs::msg::Point point;
  point.x = static_cast<double>(local.x);
  point.y = static_cast<double>(local.y);
  point.z = static_cast<double>(local.z);
  return point;
}

LocalPosition CoordinateConverter::point32ToLocal(const geometry_msgs::msg::Point32& point) const {
  LocalPosition local;
  local.x = point.x;
  local.y = point.y;
  local.z = point.z;
  return local;
}

geometry_msgs::msg::Point32 CoordinateConverter::localToPoint32(const LocalPosition& local) const {
  geometry_msgs::msg::Point32 point;
  point.x = local.x;
  point.y = local.y;
  point.z = local.z;
  return point;
}

std::vector<LocalPosition> CoordinateConverter::polygonToLocalPositions(
    const geometry_msgs::msg::Polygon& polygon) const {
  std::vector<LocalPosition> positions;
  positions.reserve(polygon.points.size());

  for (const auto& point : polygon.points) {
    positions.push_back(point32ToLocal(point));
  }

  return positions;
}

geometry_msgs::msg::Polygon CoordinateConverter::localPositionsToPolygon(
    const std::vector<LocalPosition>& positions) const {
  geometry_msgs::msg::Polygon polygon;
  polygon.points.reserve(positions.size());

  for (const auto& pos : positions) {
    polygon.points.push_back(localToPoint32(pos));
  }

  return polygon;
}

// ================================ 坐标变换矩阵 ================================

Eigen::Matrix3d CoordinateConverter::getRotationMatrix(double roll, double pitch, double yaw) const {
  double cr = std::cos(roll);
  double sr = std::sin(roll);
  double cp = std::cos(pitch);
  double sp = std::sin(pitch);
  double cy = std::cos(yaw);
  double sy = std::sin(yaw);

  Eigen::Matrix3d R;
  R << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
      -sp,   cp*sr,            cp*cr;

  return R;
}

Eigen::Matrix4d CoordinateConverter::getTransformMatrix(const LocalPosition& translation,
                                                        double roll, double pitch, double yaw) const {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  // 设置旋转部分
  T.block<3, 3>(0, 0) = getRotationMatrix(roll, pitch, yaw);

  // 设置平移部分
  T(0, 3) = translation.x;
  T(1, 3) = translation.y;
  T(2, 3) = translation.z;

  return T;
}

LocalPosition CoordinateConverter::transformPoint(const LocalPosition& point,
                                                  const Eigen::Matrix4d& transform) const {
  Eigen::Vector4d homogeneous_point(point.x, point.y, point.z, 1.0);
  Eigen::Vector4d transformed = transform * homogeneous_point;

  LocalPosition result;
  result.x = static_cast<float>(transformed.x());
  result.y = static_cast<float>(transformed.y());
  result.z = static_cast<float>(transformed.z());

  return result;
}

// ================================ 工具函数 ================================

bool CoordinateConverter::isValidLatitude(double latitude) const {
  return latitude >= -90.0 && latitude <= 90.0;
}

bool CoordinateConverter::isValidLongitude(double longitude) const {
  return longitude >= -180.0 && longitude <= 180.0;
}

bool CoordinateConverter::isValidGeodeticPosition(const GeodeticPosition& position) const {
  return isValidLatitude(position.latitude) &&
      isValidLongitude(position.longitude) &&
      std::isfinite(position.altitude);
}

double CoordinateConverter::normalizeLongitude(double longitude) const {
  while (longitude > 180.0) {
    longitude -= 360.0;
  }
  while (longitude < -180.0) {
    longitude += 360.0;
  }
  return longitude;
}

double CoordinateConverter::normalizeLatitude(double latitude) const {
  return std::max(-90.0, std::min(90.0, latitude));
}

double CoordinateConverter::normalizeAngle(double angle_deg) const {
  while (angle_deg >= 360.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg < 0.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

}  // namespace behavior_node