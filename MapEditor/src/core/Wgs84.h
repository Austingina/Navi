#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  Wgs84 — WGS84 经纬度 ↔ 局部 ENU(东-北) 平面 换算
//
//  采用与 NaviSkill/wgs84.py 一致的小范围球面近似（等距圆柱投影）：
//     e(East)  = (lon - lon0) * cos(lat0) * R * deg2rad
//     n(North) = (lat - lat0) * R * deg2rad
//  其中 R 为地球半径。适用于园区级（≲ 数公里）范围，误差可忽略。
//
//  说明：本类只做 经纬度 ↔ ENU 的平移/缩放换算，不含朝向旋转。
//  SLAM 地图的朝向（heading）在 SceneModel/SceneAlignView 里作为独立的
//  旋转项处理（geo_anchor.heading_deg，定义为 map +X 轴相对 ENU 正东、
//  逆时针为正），与设计文档 §4.4 保持一致。
// ═══════════════════════════════════════════════════════════════════════

#include <cmath>

namespace me {

class Wgs84 {
public:
    static constexpr double kEarthRadius = 6378137.0;      ///< 地球半径 (m)
    static constexpr double kDeg2Rad = 0.017453292519943295;

    Wgs84() = default;
    Wgs84(double originLon, double originLat) { setOrigin(originLon, originLat); }

    void setOrigin(double lon, double lat) {
        originLon_ = lon;
        originLat_ = lat;
        cosLat0_   = std::cos(lat * kDeg2Rad);
        valid_     = true;
    }
    bool hasOrigin() const { return valid_; }
    double originLon() const { return originLon_; }
    double originLat() const { return originLat_; }

    /// 经纬度(度) → 局部 ENU(米)。east 向东为正，north 向北为正。
    void toEnu(double lon, double lat, double& east, double& north) const {
        east  = (lon - originLon_) * kDeg2Rad * cosLat0_ * kEarthRadius;
        north = (lat - originLat_) * kDeg2Rad * kEarthRadius;
    }

    /// 局部 ENU(米) → 经纬度(度)。
    void toLonLat(double east, double north, double& lon, double& lat) const {
        lat = originLat_ + (north / kEarthRadius) / kDeg2Rad;
        double cosLat = (cosLat0_ != 0.0) ? cosLat0_ : 1.0;
        lon = originLon_ + (east / (kEarthRadius * cosLat)) / kDeg2Rad;
    }

private:
    double originLon_ = 0.0;
    double originLat_ = 0.0;
    double cosLat0_   = 1.0;
    bool   valid_     = false;
};

} // namespace me
