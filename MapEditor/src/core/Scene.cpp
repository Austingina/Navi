#include "core/Scene.h"
#include "core/Wgs84.h"

#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

namespace me {

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
    activeIndex_ = -1;
    sceneFilePath_.clear();
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
    if (o.contains("instruction")) {
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

    qDebug() << "场景加载完成:" << jsonPath
             << " maps=" << maps_.size() << " globalRoutes=" << globalRoutes_.size();
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  保存（单文件）
// ═══════════════════════════════════════════════════════════════════════
bool Scene::saveFile(const QString& jsonPath) {
    const QDir sceneDir = QFileInfo(jsonPath).absoluteDir();
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
    root["scene"] = scene;

    QJsonObject slamMaps;
    for (const auto& e : maps_) {
        QJsonObject o = e.project ? e.project->toObject() : QJsonObject{};
        o["name"] = e.name;
        o["pcd_base"] = relOrAbs(e.pcdBaseDir);
        o["ortho_visible"] = e.orthoVisible;
        QJsonObject ga;
        ga["origin"] = QJsonObject{
            {"lon", e.anchor.lon}, {"lat", e.anchor.lat}, {"alt", e.anchor.alt}};
        ga["heading_deg"] = e.anchor.headingDeg;
        ga["scale"]       = e.anchor.scale;
        o["geo_anchor"] = ga;
        // 落盘该地图的 PCD（保持点云可用）
        if (e.project && !e.pcdBaseDir.isEmpty())
            e.project->savePcds(e.pcdBaseDir);
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

    QFile file(jsonPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "无法写入场景文件:" << jsonPath;
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    sceneFilePath_ = jsonPath;
    qDebug() << "场景已保存:" << jsonPath << " maps=" << maps_.size();
    return true;
}

} // namespace me
