#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  SceneModel — 超大型场景（分层地图模型）数据模型
//
//  对应设计文档《fsm_navigation_design.md》§4「地图结构（分层地图模型）」：
//    Scene(WGS84/OSM 主干)  ─┬─  SLAM 地图 A (geo_anchor 对齐到 OSM)
//                            └─  SLAM 地图 B (geo_anchor 对齐到 OSM)
//
//  本模型负责：
//    - 保存 OSM 底图来源与经纬度范围 (scene.osm)
//    - 保存场景 ENU 原点 (scene.origin，本编辑器用于经纬度↔局部平面换算)
//    - 保存每张 SLAM 地图的引用 (source) 与 geo_anchor(lon/lat/heading)
//  与设计 §4.3 的 scene / slam_maps JSON 结构一致，并对旧版单图工程做退化兼容。
// ═══════════════════════════════════════════════════════════════════════

#include <QString>
#include <QVector>

namespace me {

/// SLAM 地图 → WGS84 的地理锚定（设计 §4.4）。
struct GeoAnchor {
    bool   valid = false;
    double lon = 0.0;          ///< map 原点(default_submap 原点)对应经度
    double lat = 0.0;          ///< map 原点对应纬度
    double alt = 0.0;          ///< 海拔 (m，可选)
    double headingDeg = 0.0;   ///< map +X 轴相对 ENU 正东的方位角(逆时针为正，度)
    double scale = 1.0;        ///< 可视对齐微调缩放(默认 1.0；SLAM 为米制，一般无需改)
};

/// 场景（OSM / WGS84）级语义点位：位置用经纬度表示，航向角以「与正北的夹角」定义。
struct SceneSemanticPoint {
    QString name;
    QString type = "single_pose";   ///< 与 SLAM 语义点位类型保持一致
    bool    needInspect = false;    ///< 是否需要巡检（默认 false）
    double  lon = 0.0;              ///< 经度
    double  lat = 0.0;              ///< 纬度
    double  alt = 0.0;              ///< 海拔 (m，可选)
    double  headingDeg = 0.0;       ///< 航向角：与正北的夹角，顺时针为正（0=正北，90=正东）
    QString area;                  ///< 所属区域名（空=未归属），用于按区域聚合管理
};

/// 场景级传送门：将一个场景语义点位定义为某张 SLAM 地图的入口。
/// 与 SLAM 地图内的传送门一致：以「语义点位」为标识，无需单独命名。
struct ScenePortal {
    QString semanticPoint;  ///< 关联的场景语义点位名称（作为传送门标识）
    QString targetSlamMap;  ///< 作为入口指向的 SLAM 地图 id
};

/// 一张 SLAM 地图在场景中的引用条目。
struct SlamMapRef {
    QString   id;              ///< 唯一标识 (slam_maps.<id>)
    QString   name;            ///< 显示名
    QString   source;          ///< SLAM 地图目录/JSON/PCD 路径 (相对 scene 目录或绝对)
    GeoAnchor anchor;
};

class SceneModel {
public:
    SceneModel() = default;

    // ── 加载 / 保存 ──────────────────────────────────────────────────
    /// 从场景 JSON 文件加载 (scene/slam_maps 结构)。
    bool load(const QString& jsonPath);
    /// 保存到场景 JSON 文件。
    bool save(const QString& jsonPath) const;

    void clear();

    // ── 场景元信息 ───────────────────────────────────────────────────
    QString name() const { return name_; }
    void setName(const QString& v) { name_ = v; }

    QString osmSource() const { return osmSource_; }
    void setOsmSource(const QString& v) { osmSource_ = v; }

    bool   hasBounds() const { return hasBounds_; }
    void setBounds(double minLon, double minLat, double maxLon, double maxLat) {
        minLon_ = minLon; minLat_ = minLat; maxLon_ = maxLon; maxLat_ = maxLat;
        hasBounds_ = true;
    }
    double minLon() const { return minLon_; }
    double minLat() const { return minLat_; }
    double maxLon() const { return maxLon_; }
    double maxLat() const { return maxLat_; }

    // 场景 ENU 原点（经纬度↔局部平面换算基准）
    bool   hasOrigin() const { return hasOrigin_; }
    void setOrigin(double lon, double lat) {
        originLon_ = lon; originLat_ = lat; hasOrigin_ = true;
    }
    double originLon() const { return originLon_; }
    double originLat() const { return originLat_; }

    // ── SLAM 地图条目 ────────────────────────────────────────────────
    QVector<SlamMapRef>&       slamMaps()       { return slamMaps_; }
    const QVector<SlamMapRef>& slamMaps() const  { return slamMaps_; }
    void addSlamMap(const SlamMapRef& m) { slamMaps_.append(m); }
    void removeSlamMapAt(int i) { if (i >= 0 && i < slamMaps_.size()) slamMaps_.remove(i); }
    /// 生成一个不冲突的 slam_map id (基于建议名)。
    QString uniqueId(const QString& suggested) const;

    QString sceneDir() const { return sceneDir_; }

private:
    QString name_ = "未命名场景";
    QString osmSource_;
    bool    hasBounds_ = false;
    double  minLon_ = 0, minLat_ = 0, maxLon_ = 0, maxLat_ = 0;
    bool    hasOrigin_ = false;
    double  originLon_ = 0, originLat_ = 0;

    QVector<SlamMapRef> slamMaps_;
    QString sceneDir_;   ///< 场景 JSON 所在目录 (用于解析相对 source)
};

} // namespace me
