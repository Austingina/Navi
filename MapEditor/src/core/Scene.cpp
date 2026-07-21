#include "core/Scene.h"
#include "core/Wgs84.h"

#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QCryptographicHash>
#include <QDebug>

namespace me {

static QByteArray jsonContentSignature(const QJsonObject& root);

void Scene::clear() {
    name_ = "未命名场景";
    osm_.reset();
    osmSource_.clear();
    hasBounds_ = false;
    minLon_ = minLat_ = maxLon_ = maxLat_ = 0.0;
    hasOrigin_ = false;
    originLon_ = originLat_ = 0.0;
    maps_.clear();
    globalRoutes_.clear();
    osmPoints_.clear();
    osmPointAreas_.clear();
    scenePortals_.clear();
    activeIndex_ = -1;
    sceneFilePath_.clear();
    buildNo_ = 0;
    contentHash_.clear();
}

SceneSemanticPoint* Scene::osmPointByName(const QString& name) {
    for (auto& p : osmPoints_)
        if (p.name == name) return &p;
    return nullptr;
}

QString Scene::uniqueOsmPointName(const QString& suggested) const {
    QString base = suggested.trimmed();
    if (base.isEmpty()) base = "point";
    auto taken = [&](const QString& n) {
        for (const auto& p : osmPoints_) if (p.name == n) return true;
        return false;
    };
    if (!taken(base)) return base;
    for (int i = 1; ; ++i) {
        QString cand = QString("%1_%2").arg(base).arg(i);
        if (!taken(cand)) return cand;
    }
}

QString Scene::uniqueAreaName(const QString& suggested) const {
    QString base = suggested.trimmed();
    if (base.isEmpty()) base = "区域";
    if (!osmPointAreas_.contains(base)) return base;
    for (int i = 1; ; ++i) {
        QString cand = QString("%1_%2").arg(base).arg(i);
        if (!osmPointAreas_.contains(cand)) return cand;
    }
}

ScenePortal* Scene::scenePortalForPoint(const QString& semanticPoint) {
    for (auto& p : scenePortals_)
        if (p.semanticPoint == semanticPoint) return &p;
    return nullptr;
}

void Scene::setOsm(std::shared_ptr<OsmData> osm, const QString& source) {
    osm_ = std::move(osm);
    osmSource_ = source;
    if (osm_ && osm_->hasBounds()) {
        minLon_ = osm_->minLon(); minLat_ = osm_->minLat();
        maxLon_ = osm_->maxLon(); maxLat_ = osm_->maxLat();
        hasBounds_ = true;
        if (!hasOrigin_) setOrigin(osm_->centerLon(), osm_->centerLat());
    }
}

int Scene::indexOfMap(const QString& id) const {
    for (int i = 0; i < maps_.size(); ++i)
        if (maps_[i].id == id) return i;
    return -1;
}

SlamMapEntry* Scene::mapById(const QString& id) {
    int i = indexOfMap(id);
    return (i >= 0) ? &maps_[i] : nullptr;
}

bool Scene::renameMapId(const QString& oldId, const QString& newId) {
    QString nid = newId.trimmed();
    nid.replace(' ', '_').replace('\t', '_');
    if (nid.isEmpty() || nid == oldId) return false;
    if (indexOfMap(oldId) < 0) return false;
    if (indexOfMap(nid) >= 0) return false;   // 与其他地图冲突

    maps_[indexOfMap(oldId)].id = nid;
    // 同步全局路线途经点中对该地图的引用
    for (auto& r : globalRoutes_)
        for (auto& wp : r.waypoints)
            if (wp.kind == GlobalWaypoint::Map && wp.slamMap == oldId)
                wp.slamMap = nid;
    // 同步场景传送门中对该地图的引用
    for (auto& p : scenePortals_)
        if (p.targetSlamMap == oldId)
            p.targetSlamMap = nid;
    return true;
}

QString Scene::uniqueMapId(const QString& suggested) const {
    QString base = suggested.trimmed();
    if (base.isEmpty()) base = "slam_map";
    base.replace(' ', '_').replace('\t', '_');
    if (indexOfMap(base) < 0) return base;
    for (int i = 1; ; ++i) {
        QString cand = QString("%1_%2").arg(base).arg(i);
        if (indexOfMap(cand) < 0) return cand;
    }
}

SlamMapEntry* Scene::activeMap() {
    if (activeIndex_ >= 0 && activeIndex_ < maps_.size()) return &maps_[activeIndex_];
    return nullptr;
}

int Scene::addMap(const SlamMapEntry& e) {
    maps_.append(e);
    activeIndex_ = maps_.size() - 1;
    return activeIndex_;
}

void Scene::removeMapAt(int i) {
    if (i < 0 || i >= maps_.size()) return;
    maps_.remove(i);
    if (activeIndex_ >= maps_.size()) activeIndex_ = maps_.size() - 1;
}

// ═══════════════════════════════════════════════════════════════════════
//  全局路线 waypoint  ↔  JSON
// ═══════════════════════════════════════════════════════════════════════
static GlobalWaypoint parseGlobalWaypoint(const QJsonObject& o) {
    GlobalWaypoint wp;
    if (o.contains("scene_point")) {
        wp.kind = GlobalWaypoint::ScenePoint;
        wp.scenePoint = o.value("scene_point").toString();
    } else if (o.contains("instruction")) {
        wp.kind = GlobalWaypoint::Instruction;
        wp.instruction = o.value("instruction").toString();
    } else if (o.contains("lon") && o.contains("lat")) {
        wp.kind = GlobalWaypoint::Gps;
        wp.lon = o.value("lon").toDouble();
        wp.lat = o.value("lat").toDouble();
    } else {
        wp.kind = GlobalWaypoint::Map;
        wp.slamMap       = o.value("slam_map").toString();
        wp.submap        = o.value("submap").toString();
        wp.semanticPoint = o.value("semantic_point").toString();
    }
    return wp;
}

static QJsonObject globalWaypointToJson(const GlobalWaypoint& wp) {
    QJsonObject o;
    switch (wp.kind) {
        case GlobalWaypoint::ScenePoint:
            o["scene_point"] = wp.scenePoint; break;
        case GlobalWaypoint::Instruction:
            o["instruction"] = wp.instruction; break;
        case GlobalWaypoint::Gps:
            o["lon"] = wp.lon; o["lat"] = wp.lat; break;
        case GlobalWaypoint::Map:
        default:
            o["slam_map"] = wp.slamMap;
            o["submap"] = wp.submap;
            o["semantic_point"] = wp.semanticPoint;
            break;
    }
    return o;
}

// ═══════════════════════════════════════════════════════════════════════
//  加载
// ═══════════════════════════════════════════════════════════════════════
bool Scene::loadFile(const QString& jsonPath) {
    QFile file(jsonPath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "无法打开场景文件:" << jsonPath;
        return false;
    }
    QJsonParseError err;
    auto doc = QJsonDocument::fromJson(file.readAll(), &err);
    if (doc.isNull()) {
        qWarning() << "场景 JSON 解析失败:" << err.errorString();
        return false;
    }

    clear();
    sceneFilePath_ = jsonPath;
    const QDir sceneDir = QFileInfo(jsonPath).absoluteDir();
    auto root = doc.object();

    // ── scene ─────────────────────────────────────────────────────────
    auto scene = root.value("scene").toObject();
    name_ = scene.value("name").toString("未命名场景");
    buildNo_ = scene.value("build_no").toVariant().toLongLong();
    if (scene.contains("origin")) {
        auto o = scene.value("origin").toObject();
        setOrigin(o.value("lon").toDouble(), o.value("lat").toDouble());
    }
    auto osmObj = scene.value("osm").toObject();
    QString osmSrc = osmObj.value("source").toString();
    if (osmObj.contains("bounds")) {
        auto b = osmObj.value("bounds").toObject();
        auto mn = b.value("min").toObject(), mx = b.value("max").toObject();
        minLon_ = mn.value("lon").toDouble(); minLat_ = mn.value("lat").toDouble();
        maxLon_ = mx.value("lon").toDouble(); maxLat_ = mx.value("lat").toDouble();
        hasBounds_ = true;
    }
    if (!osmSrc.isEmpty()) {
        QString abs = QFileInfo(osmSrc).isAbsolute() ? osmSrc
                                                     : sceneDir.absoluteFilePath(osmSrc);
        auto osm = std::make_shared<OsmData>();
        if (osm->load(abs) && !osm->isEmpty()) {
            osm_ = osm;
            osmSource_ = abs;
            if (!hasOrigin_ && osm_->hasBounds())
                setOrigin(osm_->centerLon(), osm_->centerLat());
        } else {
            qWarning() << "场景底图加载失败:" << abs;
        }
    }

    // ── scene.semantic_points（OSM 底图上的场景语义点位）────────────────
    {
        auto spObj = scene.value("semantic_points").toObject();
        for (auto it = spObj.begin(); it != spObj.end(); ++it) {
            auto o = it.value().toObject();
            SceneSemanticPoint p;
            p.name        = it.key();
            p.type        = o.value("type").toString("single_pose");
            p.needInspect = o.value("need_inspect").toBool(false);
            p.lon         = o.value("lon").toDouble();
            p.lat         = o.value("lat").toDouble();
            p.alt         = o.value("alt").toDouble();
            p.headingDeg  = o.value("heading_deg").toDouble();
            // 兼容旧字段名 group
            p.area        = o.value("area").toString(o.value("group").toString());
            osmPoints_.append(p);
        }
        // 区域列表（含空区域）；再并入点位里出现过的区域名，保证不丢
        // 兼容旧键名 semantic_point_groups
        QJsonArray areaArr = scene.value("semantic_point_areas").toArray();
        if (areaArr.isEmpty()) areaArr = scene.value("semantic_point_groups").toArray();
        for (const auto& g : areaArr)
            if (!g.toString().isEmpty() && !osmPointAreas_.contains(g.toString()))
                osmPointAreas_.append(g.toString());
        for (const auto& p : osmPoints_)
            if (!p.area.isEmpty() && !osmPointAreas_.contains(p.area))
                osmPointAreas_.append(p.area);
    }

    // ── scene.portals（场景级传送门：语义点位 → SLAM 地图入口）──────────
    {
        auto ptObj = scene.value("portals").toObject();
        for (auto it = ptObj.begin(); it != ptObj.end(); ++it) {
            auto o = it.value().toObject();
            ScenePortal p;
            // 新格式：以语义点位为键；旧格式：以传送门名为键、内含 semantic_point
            p.semanticPoint = o.value("semantic_point").toString(it.key());
            p.targetSlamMap = o.value("target_slam_map").toString();
            scenePortals_.append(p);
        }
    }

    // ── slam_maps ──────────────────────────────────────────────────────
    auto slamMaps = root.value("slam_maps").toObject();
    for (auto it = slamMaps.begin(); it != slamMaps.end(); ++it) {
        auto o = it.value().toObject();
        SlamMapEntry e;
        e.id   = it.key();
        e.name = o.value("name").toString(it.key());
        if (o.contains("geo_anchor")) {
            auto ga = o.value("geo_anchor").toObject();
            auto origin = ga.value("origin").toObject();
            e.anchor.lon = origin.value("lon").toDouble();
            e.anchor.lat = origin.value("lat").toDouble();
            e.anchor.alt = origin.value("alt").toDouble();
            e.anchor.headingDeg = ga.value("heading_deg").toDouble();
            e.anchor.scale = ga.value("scale").toDouble(1.0);
            e.anchor.valid = true;
        }
        e.orthoVisible = o.value("ortho_visible").toBool(true);
        e.locked = o.value("locked").toBool(false);
        QString pcdBase = o.value("pcd_base").toString();
        e.pcdBaseDir = pcdBase.isEmpty()
                           ? sceneDir.absolutePath()
                           : (QFileInfo(pcdBase).isAbsolute()
                                  ? pcdBase
                                  : sceneDir.absoluteFilePath(pcdBase));
        e.project = std::make_shared<MapProject>();
        e.project->loadFromObject(o, e.pcdBaseDir);
        maps_.append(e);
    }
    if (!maps_.isEmpty()) activeIndex_ = 0;

    // ── 全局 routes ─────────────────────────────────────────────────────
    auto routes = root.value("routes").toObject();
    for (auto it = routes.begin(); it != routes.end(); ++it) {
        auto o = it.value().toObject();
        GlobalRoute r;
        r.name = it.key();
        r.description = o.value("description").toString();
        for (const auto& wv : o.value("waypoints").toArray())
            r.waypoints.append(parseGlobalWaypoint(wv.toObject()));
        globalRoutes_.append(r);
    }

    // 记录加载时的内容指纹：后续保存若内容未变则保持 scene.build_no 不变
    contentHash_ = jsonContentSignature(buildRootObject(sceneDir.absolutePath()));

    qDebug() << "场景加载完成:" << jsonPath
             << " maps=" << maps_.size() << " globalRoutes=" << globalRoutes_.size()
             << " build_no=" << buildNo_;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  内容指纹：递归剔除所有 build_no 后哈希（保证时间戳本身不参与比较）
// ═══════════════════════════════════════════════════════════════════════
static QJsonValue stripBuildNoRecursive(const QJsonValue& v) {
    if (v.isObject()) {
        QJsonObject o = v.toObject();
        o.remove("build_no");
        for (auto it = o.begin(); it != o.end(); ++it)
            *it = stripBuildNoRecursive(it.value());
        return o;
    }
    if (v.isArray()) {
        QJsonArray a = v.toArray();
        for (int i = 0; i < a.size(); ++i)
            a[i] = stripBuildNoRecursive(a[i]);
        return a;
    }
    return v;
}

static QByteArray jsonContentSignature(const QJsonObject& root) {
    const QJsonValue cleaned = stripBuildNoRecursive(root);
    const QByteArray bytes = QJsonDocument(cleaned.toObject())
                                 .toJson(QJsonDocument::Compact);
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha1);
}

// ═══════════════════════════════════════════════════════════════════════
//  组装 root 对象（不含 scene.build_no；slam_maps 各自携带自身 build_no）
// ═══════════════════════════════════════════════════════════════════════
QJsonObject Scene::buildRootObject(const QString& baseDir) const {
    const QDir sceneDir(baseDir);
    auto relOrAbs = [&](const QString& abs) -> QString {
        if (abs.isEmpty()) return abs;
        QString rel = sceneDir.relativeFilePath(abs);
        return rel.startsWith("..") ? abs : rel;   // 场景目录之外则保留绝对
    };

    QJsonObject root;

    QJsonObject scene;
    scene["name"] = name_;
    scene["crs"]  = "WGS84";
    if (hasOrigin_)
        scene["origin"] = QJsonObject{{"lon", originLon_}, {"lat", originLat_}};
    QJsonObject osmObj;
    osmObj["source"] = relOrAbs(osmSource_);
    if (hasBounds_) {
        osmObj["bounds"] = QJsonObject{
            {"min", QJsonObject{{"lon", minLon_}, {"lat", minLat_}}},
            {"max", QJsonObject{{"lon", maxLon_}, {"lat", maxLat_}}}};
    }
    scene["osm"] = osmObj;
    if (!osmPoints_.isEmpty()) {
        QJsonObject spObj;
        for (const auto& p : osmPoints_) {
            QJsonObject o;
            o["type"]         = p.type;
            o["need_inspect"] = p.needInspect;
            o["lon"]          = p.lon;
            o["lat"]          = p.lat;
            o["alt"]          = p.alt;
            o["heading_deg"]  = p.headingDeg;
            if (!p.area.isEmpty()) o["area"] = p.area;
            spObj[p.name] = o;
        }
        scene["semantic_points"] = spObj;
    }
    if (!osmPointAreas_.isEmpty()) {
        QJsonArray garr;
        for (const auto& g : osmPointAreas_) garr.append(g);
        scene["semantic_point_areas"] = garr;
    }
    if (!scenePortals_.isEmpty()) {
        QJsonObject ptObj;
        for (const auto& p : scenePortals_) {
            QJsonObject o;
            o["target_slam_map"] = p.targetSlamMap;
            ptObj[p.semanticPoint] = o;   // 以语义点位为键
        }
        scene["portals"] = ptObj;
    }
    root["scene"] = scene;

    QJsonObject slamMaps;
    for (const auto& e : maps_) {
        QJsonObject o = e.project ? e.project->toObject() : QJsonObject{};
        o["name"] = e.name;
        o["pcd_base"] = relOrAbs(e.pcdBaseDir);
        o["ortho_visible"] = e.orthoVisible;
        o["locked"] = e.locked;
        QJsonObject ga;
        ga["origin"] = QJsonObject{
            {"lon", e.anchor.lon}, {"lat", e.anchor.lat}, {"alt", e.anchor.alt}};
        ga["heading_deg"] = e.anchor.headingDeg;
        ga["scale"]       = e.anchor.scale;
        o["geo_anchor"] = ga;
        // 注意：PCD 的落盘/复制由 saveToDir() 负责（此处只写 JSON 引用）。
        slamMaps[e.id] = o;
    }
    root["slam_maps"] = slamMaps;

    QJsonObject routes;
    for (const auto& r : globalRoutes_) {
        QJsonObject o;
        o["description"] = r.description;
        QJsonArray arr;
        for (const auto& wp : r.waypoints) arr.append(globalWaypointToJson(wp));
        o["waypoints"] = arr;
        routes[r.name] = o;
    }
    root["routes"] = routes;

    return root;
}

// ═══════════════════════════════════════════════════════════════════════
//  保存（单文件）
// ═══════════════════════════════════════════════════════════════════════
bool Scene::saveFile(const QString& jsonPath) {
    const QString baseDir = QFileInfo(jsonPath).absolutePath();

    // 1. 各 SLAM 地图：内部信息有更新才刷新自身 global.build_no
    for (const auto& e : maps_)
        if (e.project) e.project->refreshBuildNoIfChanged();

    // 2. 组装 root（此时 slam_maps 已带最新 build_no，scene 尚无 build_no）
    QJsonObject root = buildRootObject(baseDir);

    // 3. 场景级 build_no：整个场景内容（剔除所有 build_no）有变化才刷新时间戳
    const QByteArray sig = jsonContentSignature(root);
    if (buildNo_ == 0 || sig != contentHash_) {
        buildNo_ = QDateTime::currentDateTime()
                       .toString(QStringLiteral("yyyyMMddhhmmss"))
                       .toLongLong();
        contentHash_ = sig;
    }
    QJsonObject scene = root.value("scene").toObject();
    scene["build_no"] = QJsonValue(static_cast<qint64>(buildNo_));
    root["scene"] = scene;

    QFile file(jsonPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "无法写入场景文件:" << jsonPath;
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    sceneFilePath_ = jsonPath;
    qDebug() << "场景已保存:" << jsonPath << " maps=" << maps_.size()
             << " build_no=" << buildNo_;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  目录级 保存 / 打开（自包含目录，资源相对路径引用，便于迁移）
// ═══════════════════════════════════════════════════════════════════════
static bool copyFileOverwrite(const QString& src, const QString& dst) {
    if (src.isEmpty() || dst.isEmpty()) return false;
    if (QFileInfo(src).absoluteFilePath() == QFileInfo(dst).absoluteFilePath())
        return true;   // 同一文件，无需复制
    QDir().mkpath(QFileInfo(dst).absolutePath());
    if (QFile::exists(dst)) QFile::remove(dst);
    if (!QFile::copy(src, dst)) {
        qWarning() << "复制文件失败:" << src << "->" << dst;
        return false;
    }
    return true;
}

bool Scene::saveToDir(const QString& dir) {
    if (dir.isEmpty()) return false;
    QDir().mkpath(dir);
    const QDir sceneDir(dir);

    // 1) OSM/PBF 底图 → <dir>/osm/<filename>
    if (osm_ && !osmSource_.isEmpty()) {
        const QString name = QFileInfo(osmSource_).fileName();
        if (!name.isEmpty() && QFile::exists(osmSource_)) {
            const QString dst = sceneDir.absoluteFilePath("osm/" + name);
            if (copyFileOverwrite(osmSource_, dst))
                osmSource_ = dst;   // 指向目录内副本；saveFile 里会写成相对路径
        }
    }

    // 2) 各 SLAM 地图的 PCD → <dir>/<mapId>/<map_file>
    for (auto& e : maps_) {
        if (!e.project) continue;
        const QString newBase = sceneDir.absoluteFilePath(e.id);
        QDir().mkpath(newBase);
        const QString oldBase = e.pcdBaseDir;
        bool anyMissing = false;
        for (const auto& sid : e.project->submapIds()) {
            auto* sm = e.project->submap(sid);
            if (!sm || sm->mapFile().isEmpty()) continue;
            const QString dst = QDir(newBase).absoluteFilePath(sm->mapFile());
            const QString src = oldBase.isEmpty()
                                    ? QString()
                                    : QDir(oldBase).absoluteFilePath(sm->mapFile());
            if (!src.isEmpty() && QFile::exists(src))
                copyFileOverwrite(src, dst);          // 直接复制原始 PCD（快速、无损）
            else
                anyMissing = true;                    // 源文件缺失，稍后用内存点云兜底
        }
        // 兜底：源文件缺失时，用内存点云重新落盘到目录内
        if (anyMissing) e.project->savePcds(newBase);
        e.pcdBaseDir = newBase;
    }

    // 3) 写 scene.json（资源引用一律相对路径）
    return saveFile(sceneDir.absoluteFilePath("scene.json"));
}

bool Scene::loadFromDir(const QString& dir) {
    const QString jsonPath = QDir(dir).absoluteFilePath("scene.json");
    if (!QFile::exists(jsonPath)) {
        qWarning() << "目录中未找到 scene.json:" << dir;
        return false;
    }
    return loadFile(jsonPath);
}

QString Scene::sceneDir() const {
    return sceneFilePath_.isEmpty() ? QString()
                                    : QFileInfo(sceneFilePath_).absolutePath();
}

} // namespace me
