#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  Scene — 分层地图场景（可编辑）：OSM 底图 + 多张 SLAM 地图 + 全局路线
//
//  对应设计文档 §4「地图结构（分层地图模型）」：
//    · OSM 底图作为场景 base layer（经纬度主干）
//    · 多张 SLAM 地图各带 geo_anchor 锚定到 OSM；每张内部仍是多 submap 结构
//    · 全局路线可跨多张 SLAM 地图 / OSM；内部路线保存在各 SLAM 地图自身
//
//  持久化：单文件 JSON（scene + slam_maps 内联 + 全局 routes）；OSM 文件与各
//  submap 的 PCD 为二进制资源，仍按路径引用（map_file / osm.source）。
// ═══════════════════════════════════════════════════════════════════════

#include "core/SceneModel.h"   // GeoAnchor
#include "core/OsmData.h"
#include "core/MapProject.h"

#include <QString>
#include <QVector>
#include <memory>

namespace me {

/// 一张 SLAM 地图在场景中的可编辑条目。
struct SlamMapEntry {
    QString   id;
    QString   name;
    GeoAnchor anchor;
    QString   pcdBaseDir;   ///< 解析各 submap map_file 的基准目录
    std::shared_ptr<MapProject> project;   ///< submaps + 内部路线（可编辑）
    bool      orthoVisible = true;         ///< 场景总览里是否显示该地图的正交投影
};

/// 全局路线途经点（异构：map / gps / instruction）。
struct GlobalWaypoint {
    enum Kind { Map, Gps, Instruction };
    Kind    kind = Map;
    // map
    QString slamMap, submap, semanticPoint;
    // gps
    double  lon = 0.0, lat = 0.0;
    // instruction
    QString instruction;
};

struct GlobalRoute {
    QString name;
    QString description;
    QVector<GlobalWaypoint> waypoints;
};

class Scene {
public:
    Scene() = default;

    void clear();

    // ── OSM 底图 ─────────────────────────────────────────────────────
    bool hasOsm() const { return osm_ != nullptr; }
    std::shared_ptr<OsmData> osm() const { return osm_; }
    QString osmSource() const { return osmSource_; }
    void setOsm(std::shared_ptr<OsmData> osm, const QString& source);

    // ── 场景 ENU 原点 ────────────────────────────────────────────────
    bool   hasOrigin() const { return hasOrigin_; }
    double originLon() const { return originLon_; }
    double originLat() const { return originLat_; }
    void   setOrigin(double lon, double lat) { originLon_ = lon; originLat_ = lat; hasOrigin_ = true; }

    QString name() const { return name_; }
    void setName(const QString& n) { name_ = n; }

    // ── SLAM 地图 ────────────────────────────────────────────────────
    QVector<SlamMapEntry>&       maps()       { return maps_; }
    const QVector<SlamMapEntry>& maps() const  { return maps_; }
    int  mapCount() const { return maps_.size(); }
    int  indexOfMap(const QString& id) const;
    SlamMapEntry* mapById(const QString& id);
    QString uniqueMapId(const QString& suggested) const;

    int  activeIndex() const { return activeIndex_; }
    void setActiveIndex(int i) { activeIndex_ = i; }
    SlamMapEntry* activeMap();

    /// 追加一张 SLAM 地图（project 已就绪）。返回其索引。
    int addMap(const SlamMapEntry& e);
    void removeMapAt(int i);

    // ── 全局路线 ─────────────────────────────────────────────────────
    QVector<GlobalRoute>&       globalRoutes()       { return globalRoutes_; }
    const QVector<GlobalRoute>& globalRoutes() const  { return globalRoutes_; }

    // ── 持久化（单文件）─────────────────────────────────────────────
    bool loadFile(const QString& jsonPath);
    bool saveFile(const QString& jsonPath);
    QString sceneFilePath() const { return sceneFilePath_; }

private:
    QString name_ = "未命名场景";
    std::shared_ptr<OsmData> osm_;
    QString osmSource_;
    bool    hasBounds_ = false;
    double  minLon_ = 0, minLat_ = 0, maxLon_ = 0, maxLat_ = 0;
    bool    hasOrigin_ = false;
    double  originLon_ = 0, originLat_ = 0;

    QVector<SlamMapEntry> maps_;
    QVector<GlobalRoute>  globalRoutes_;
    int activeIndex_ = -1;
    QString sceneFilePath_;
};

} // namespace me
