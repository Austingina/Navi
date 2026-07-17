#include "core/SceneModel.h"

#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

namespace me {

void SceneModel::clear() {
    name_ = "未命名场景";
    osmSource_.clear();
    hasBounds_ = false;
    minLon_ = minLat_ = maxLon_ = maxLat_ = 0.0;
    hasOrigin_ = false;
    originLon_ = originLat_ = 0.0;
    slamMaps_.clear();
    sceneDir_.clear();
}

QString SceneModel::uniqueId(const QString& suggested) const {
    QString base = suggested.trimmed();
    if (base.isEmpty()) base = "slam_map";
    // 替换空白为下划线
    base.replace(' ', '_').replace('\t', '_');

    auto exists = [this](const QString& id) {
        for (const auto& m : slamMaps_)
            if (m.id == id) return true;
        return false;
    };
    if (!exists(base)) return base;
    for (int i = 1; ; ++i) {
        QString cand = QString("%1_%2").arg(base).arg(i);
        if (!exists(cand)) return cand;
    }
}

// ── 加载 ─────────────────────────────────────────────────────────────
bool SceneModel::load(const QString& jsonPath) {
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
    sceneDir_ = QFileInfo(jsonPath).absolutePath();

    auto root = doc.object();
    auto scene = root.value("scene").toObject();
    name_ = scene.value("name").toString("未命名场景");

    if (scene.contains("origin")) {
        auto o = scene.value("origin").toObject();
        originLon_ = o.value("lon").toDouble();
        originLat_ = o.value("lat").toDouble();
        hasOrigin_ = true;
    }

    auto osm = scene.value("osm").toObject();
    osmSource_ = osm.value("source").toString();
    if (osm.contains("bounds")) {
        auto b = osm.value("bounds").toObject();
        auto mn = b.value("min").toObject();
        auto mx = b.value("max").toObject();
        minLon_ = mn.value("lon").toDouble();
        minLat_ = mn.value("lat").toDouble();
        maxLon_ = mx.value("lon").toDouble();
        maxLat_ = mx.value("lat").toDouble();
        hasBounds_ = true;
    }
    // 若未显式给 origin，则用 bounds 中心
    if (!hasOrigin_ && hasBounds_) {
        originLon_ = 0.5 * (minLon_ + maxLon_);
        originLat_ = 0.5 * (minLat_ + maxLat_);
        hasOrigin_ = true;
    }

    auto slamMaps = root.value("slam_maps").toObject();
    for (auto it = slamMaps.begin(); it != slamMaps.end(); ++it) {
        auto o = it.value().toObject();
        SlamMapRef m;
        m.id     = it.key();
        m.name   = o.value("name").toString(it.key());
        m.source = o.value("source").toString();
        if (o.contains("geo_anchor")) {
            auto ga = o.value("geo_anchor").toObject();
            auto origin = ga.value("origin").toObject();
            m.anchor.lon = origin.value("lon").toDouble();
            m.anchor.lat = origin.value("lat").toDouble();
            m.anchor.alt = origin.value("alt").toDouble();
            m.anchor.headingDeg = ga.value("heading_deg").toDouble();
            m.anchor.scale = ga.value("scale").toDouble(1.0);
            m.anchor.valid = true;
        }
        slamMaps_.append(m);
    }

    qDebug() << "场景加载完成:" << jsonPath
             << " slam_maps=" << slamMaps_.size();
    return true;
}

// ── 保存 ─────────────────────────────────────────────────────────────
bool SceneModel::save(const QString& jsonPath) const {
    QJsonObject root;
    root["_comment"] = QString(
        "分层地图场景配置 (scene + slam_maps)，见 fsm_navigation_design.md §4");

    // ── scene ────────────────────────────────────────────────────────
    QJsonObject scene;
    scene["name"] = name_;
    scene["crs"]  = "WGS84";
    if (hasOrigin_) {
        scene["origin"] = QJsonObject{{"lon", originLon_}, {"lat", originLat_}};
    }
    QJsonObject osm;
    osm["source"] = osmSource_;
    if (hasBounds_) {
        osm["bounds"] = QJsonObject{
            {"min", QJsonObject{{"lon", minLon_}, {"lat", minLat_}}},
            {"max", QJsonObject{{"lon", maxLon_}, {"lat", maxLat_}}}};
    }
    scene["osm"] = osm;
    root["scene"] = scene;

    // ── slam_maps ─────────────────────────────────────────────────────
    QJsonObject slamMaps;
    for (const auto& m : slamMaps_) {
        QJsonObject o;
        o["name"]   = m.name;
        o["source"] = m.source;
        QJsonObject ga;
        ga["origin"] = QJsonObject{
            {"lon", m.anchor.lon}, {"lat", m.anchor.lat}, {"alt", m.anchor.alt}};
        ga["heading_deg"] = m.anchor.headingDeg;
        ga["scale"]       = m.anchor.scale;
        o["geo_anchor"] = ga;
        slamMaps[m.id] = o;
    }
    root["slam_maps"] = slamMaps;

    QFile file(jsonPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "无法写入场景文件:" << jsonPath;
        return false;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();
    qDebug() << "场景已保存:" << jsonPath;
    return true;
}

} // namespace me
