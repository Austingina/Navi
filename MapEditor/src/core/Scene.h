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
#include <QStringList>
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
    bool      locked = false;              ///< 锁定后在场景总览中拖拽 = 平移视图，不改动 geo_anchor
};

/// 全局路线途经点（异构：map / gps / instruction / scene_point）。
struct GlobalWaypoint {
    enum Kind { Map, Gps, Instruction, ScenePoint };
    Kind    kind = Map;
    // map
    QString slamMap, submap, semanticPoint;
    // gps
    double  lon = 0.0, lat = 0.0;
    // instruction
    QString instruction;
    // scene_point：引用 OSM 底图上的场景语义点位（按名称）
    QString scenePoint;
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
    /// 修改某张 SLAM 地图的 id（code），并同步更新全局路线途经点、场景传送门中的引用。
    /// newId 为空、与其他地图冲突或 oldId 不存在时返回 false（不做任何修改）。
    bool renameMapId(const QString& oldId, const QString& newId);

    int  activeIndex() const { return activeIndex_; }
    void setActiveIndex(int i) { activeIndex_ = i; }
    SlamMapEntry* activeMap();

    /// 追加一张 SLAM 地图（project 已就绪）。返回其索引。
    int addMap(const SlamMapEntry& e);
    void removeMapAt(int i);

    // ── 全局路线 ─────────────────────────────────────────────────────
    QVector<GlobalRoute>&       globalRoutes()       { return globalRoutes_; }
    const QVector<GlobalRoute>& globalRoutes() const  { return globalRoutes_; }

    // ── OSM 底图上的场景语义点位（经纬度 + 航向角）───────────────────
    QVector<SceneSemanticPoint>&       osmPoints()       { return osmPoints_; }
    const QVector<SceneSemanticPoint>& osmPoints() const  { return osmPoints_; }
    SceneSemanticPoint* osmPointByName(const QString& name);
    /// 生成一个不冲突的点位名（基于建议名）。
    QString uniqueOsmPointName(const QString& suggested) const;

    // ── 语义点位区域（按区域聚合管理）────────────────────────────────
    QStringList&       osmPointAreas()       { return osmPointAreas_; }
    const QStringList& osmPointAreas() const  { return osmPointAreas_; }
    QString uniqueAreaName(const QString& suggested) const;

    // ── 场景级传送门（语义点位 → SLAM 地图入口）──────────────────────
    QVector<ScenePortal>&       scenePortals()       { return scenePortals_; }
    const QVector<ScenePortal>& scenePortals() const  { return scenePortals_; }
    /// 按关联的语义点位查找传送门（语义点位即传送门标识）。
    ScenePortal* scenePortalForPoint(const QString& semanticPoint);

    // ── 持久化 ───────────────────────────────────────────────────────
    /// 以「场景目录」为操作对象：目录内含 scene.json + 复制进来的 OSM/PBF + 各地图 PCD；
    /// scene.json 内的资源引用一律用相对路径，方便整目录迁移。
    bool saveToDir(const QString& dir);
    bool loadFromDir(const QString& dir);
    /// 场景所在目录（已保存/打开过才有效）。
    QString sceneDir() const;

    // 低层：直接读写指定的 scene.json（相对路径以其所在目录解析）。
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
    QVector<SceneSemanticPoint> osmPoints_;
    QStringList           osmPointAreas_;
    QVector<ScenePortal>  scenePortals_;
    int activeIndex_ = -1;
    QString sceneFilePath_;
};

} // namespace me
