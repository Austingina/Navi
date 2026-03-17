#include "core/MapProject.h"

#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

#include <pcl/io/pcd_io.h>

namespace me {

// ═══════════════════════════════════════════════════════════════════════
//  JSON → 内部数据结构
// ═══════════════════════════════════════════════════════════════════════

Pose MapProject::parsePose(const QJsonObject& obj) const {
    Pose p;
    if (obj.contains("position")) {
        auto pos = obj["position"].toObject();
        p.position = {pos["x"].toDouble(), pos["y"].toDouble(), pos["z"].toDouble()};
    }
    if (obj.contains("orientation")) {
        auto ori = obj["orientation"].toObject();
        p.orientation = Eigen::Quaterniond(
            ori["w"].toDouble(), ori["x"].toDouble(),
            ori["y"].toDouble(), ori["z"].toDouble());
    }
    return p;
}

RigidTransform MapProject::parseTransform(const QJsonObject& obj) const {
    RigidTransform t;
    if (obj.contains("translation")) {
        auto tr = obj["translation"].toObject();
        t.translation = {tr["x"].toDouble(), tr["y"].toDouble(), tr["z"].toDouble()};
    }
    if (obj.contains("rotation")) {
        auto ro = obj["rotation"].toObject();
        t.rotation = Eigen::Quaterniond(
            ro["w"].toDouble(), ro["x"].toDouble(),
            ro["y"].toDouble(), ro["z"].toDouble());
    }
    return t;
}

// ═══════════════════════════════════════════════════════════════════════
//  内部数据结构 → JSON
// ═══════════════════════════════════════════════════════════════════════

QJsonObject MapProject::poseToJson(const Pose& p) const {
    QJsonObject obj;
    obj["position"] = QJsonObject{
        {"x", p.position.x()}, {"y", p.position.y()}, {"z", p.position.z()}};
    obj["orientation"] = QJsonObject{
        {"x", p.orientation.x()}, {"y", p.orientation.y()},
        {"z", p.orientation.z()}, {"w", p.orientation.w()}};
    return obj;
}

QJsonObject MapProject::transformToJson(const RigidTransform& t) const {
    QJsonObject obj;
    obj["translation"] = QJsonObject{
        {"x", t.translation.x()}, {"y", t.translation.y()}, {"z", t.translation.z()}};
    obj["rotation"] = QJsonObject{
        {"x", t.rotation.x()}, {"y", t.rotation.y()},
        {"z", t.rotation.z()}, {"w", t.rotation.w()}};
    return obj;
}

// ═══════════════════════════════════════════════════════════════════════
//  PCD 文件 I/O
// ═══════════════════════════════════════════════════════════════════════

CloudPtr MapProject::loadPcd(const QString& path) const {
    auto cloud = std::make_shared<CloudT>();
    if (pcl::io::loadPCDFile<PointT>(path.toStdString(), *cloud) == -1) {
        qWarning() << "无法加载 PCD 文件:" << path;
        return nullptr;
    }
    qDebug() << "已加载点云:" << path << " 点数:" << cloud->size();
    return cloud;
}

bool MapProject::savePcd(const QString& path, CloudPtr cloud) const {
    if (!cloud) return false;
    if (pcl::io::savePCDFileBinary(path.toStdString(), *cloud) == -1) {
        qWarning() << "无法保存 PCD 文件:" << path;
        return false;
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  加载 multifloor.json
// ═══════════════════════════════════════════════════════════════════════

bool MapProject::load(const QString& jsonPath) {
    QFile file(jsonPath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "无法打开文件:" << jsonPath;
        return false;
    }

    QJsonParseError err;
    auto doc = QJsonDocument::fromJson(file.readAll(), &err);
    if (doc.isNull()) {
        qWarning() << "JSON 解析失败:" << err.errorString();
        return false;
    }

    auto root = doc.object();
    projectDir_ = QFileInfo(jsonPath).absolutePath();

    // ── global ───────────────────────────────────────────────────────
    if (root.contains("global")) {
        auto global = root["global"].toObject();
        defaultSubmapId_ = global["default_submap"].toString("floor");
    }

    // ── submaps ──────────────────────────────────────────────────────
    submaps_.clear();
    if (root.contains("submaps")) {
        auto submapsObj = root["submaps"].toObject();
        for (auto it = submapsObj.begin(); it != submapsObj.end(); ++it) {
            QString smId = it.key();
            auto smObj = it.value().toObject();

            auto sm = std::make_shared<SubMap>(smId);
            sm->setName(smObj["name"].toString());
            sm->setDescription(smObj["description"].toString());
            sm->setMapFile(smObj["map_file"].toString());

            // transform
            if (smObj.contains("transform")) {
                sm->setTransform(parseTransform(smObj["transform"].toObject()));
            }

            // submap_portals
            if (smObj.contains("submap_portals")) {
                auto arr = smObj["submap_portals"].toArray();
                for (const auto& val : arr) {
                    auto pObj = val.toObject();
                    Portal portal;
                    portal.portalName   = pObj["portal"].toString();
                    portal.targetSubmap = pObj["submap"].toString();
                    sm->addPortal(portal);
                }
            }

            // semantic_points
            if (smObj.contains("semantic_points")) {
                auto spObj = smObj["semantic_points"].toObject();
                for (auto sit = spObj.begin(); sit != spObj.end(); ++sit) {
                    auto spData = sit.value().toObject();
                    SemanticPoint sp;
                    sp.name = sit.key();
                    sp.type = spData["type"].toString("single_pose");

                    if (sp.type == "multi_pose" && spData.contains("poses")) {
                        auto posesArr = spData["poses"].toArray();
                        for (const auto& pv : posesArr) {
                            sp.poses.append(parsePose(pv.toObject()));
                        }
                    } else if (spData.contains("pose")) {
                        sp.poses.append(parsePose(spData["pose"].toObject()));
                    }

                    sm->addSemanticPoint(sp);
                }
            }

            // 加载 PCD
            if (!sm->mapFile().isEmpty()) {
                QString pcdPath = projectDir_ + "/" + sm->mapFile();
                auto cloud = loadPcd(pcdPath);
                if (cloud) sm->setCloud(cloud);
            }

            submaps_[smId] = sm;
        }
    }

    // ── routes ───────────────────────────────────────────────────────
    routes_.clear();
    if (root.contains("routes")) {
        auto routesObj = root["routes"].toObject();
        for (auto it = routesObj.begin(); it != routesObj.end(); ++it) {
            auto rObj = it.value().toObject();
            Route route;
            route.name = it.key();
            route.description = rObj["description"].toString();

            if (rObj.contains("waypoints")) {
                auto wpArr = rObj["waypoints"].toArray();
                for (const auto& wv : wpArr) {
                    auto wObj = wv.toObject();
                    RouteWaypoint wp;
                    wp.submapId      = wObj["submap"].toString();
                    wp.semanticPoint = wObj["semantic_point"].toString();
                    route.waypoints.append(wp);
                }
            }
            routes_[route.name] = route;
        }
    }

    dirty_ = false;
    qDebug() << "地图工程加载完成. Submaps:" << submaps_.size()
             << " Routes:" << routes_.size();
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  保存 multifloor.json
// ═══════════════════════════════════════════════════════════════════════

bool MapProject::save(const QString& jsonPath) const {
    QJsonObject root;

    // ── global ───────────────────────────────────────────────────────
    root["_comment"] = QString("多楼层地图配置文件 / Multi-floor map configuration");
    root["global"] = QJsonObject{{"default_submap", defaultSubmapId_}};

    // ── submaps ──────────────────────────────────────────────────────
    QJsonObject submapsObj;
    for (auto it = submaps_.begin(); it != submaps_.end(); ++it) {
        const auto& sm = it.value();
        QJsonObject smObj;
        smObj["name"]        = sm->name();
        smObj["description"] = sm->description();
        smObj["map_file"]    = sm->mapFile();
        smObj["transform"]   = transformToJson(sm->transform());

        // submap_portals
        QJsonArray portalsArr;
        for (const auto& p : sm->portals()) {
            portalsArr.append(QJsonObject{
                {"portal", p.portalName},
                {"submap", p.targetSubmap}
            });
        }
        smObj["submap_portals"] = portalsArr;

        // semantic_points
        QJsonObject spObj;
        for (auto sit = sm->semanticPoints().begin();
             sit != sm->semanticPoints().end(); ++sit) {
            const auto& sp = sit.value();
            QJsonObject spData;
            spData["type"] = sp.type;
            if (sp.type == "multi_pose") {
                QJsonArray posesArr;
                for (const auto& pose : sp.poses) {
                    posesArr.append(poseToJson(pose));
                }
                spData["poses"] = posesArr;
            } else if (!sp.poses.isEmpty()) {
                spData["pose"] = poseToJson(sp.poses.first());
            }
            spObj[sp.name] = spData;
        }
        smObj["semantic_points"] = spObj;

        submapsObj[it.key()] = smObj;
    }
    root["submaps"] = submapsObj;

    // ── routes ───────────────────────────────────────────────────────
    QJsonObject routesObj;
    for (auto it = routes_.begin(); it != routes_.end(); ++it) {
        const auto& r = it.value();
        QJsonObject rObj;
        rObj["description"] = r.description;
        QJsonArray wpArr;
        for (const auto& wp : r.waypoints) {
            wpArr.append(QJsonObject{
                {"submap", wp.submapId},
                {"semantic_point", wp.semanticPoint}
            });
        }
        rObj["waypoints"] = wpArr;
        routesObj[it.key()] = rObj;
    }
    root["routes"] = routesObj;

    // ── 写文件 ───────────────────────────────────────────────────────
    QString saveDir = QFileInfo(jsonPath).absolutePath();
    QDir().mkpath(saveDir);

    QFile file(jsonPath);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "无法写入文件:" << jsonPath;
        return false;
    }

    QJsonDocument doc(root);
    file.write(doc.toJson(QJsonDocument::Indented));
    file.close();

    // 保存各 submap 的 PCD (原始精度)
    for (auto it = submaps_.begin(); it != submaps_.end(); ++it) {
        const auto& sm = it.value();
        if (sm->cloud() && !sm->mapFile().isEmpty()) {
            QString pcdPath = saveDir + "/" + sm->mapFile();
            savePcd(pcdPath, sm->cloud());
        }
    }

    qDebug() << "地图工程已保存至:" << jsonPath;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Submap 管理
// ═══════════════════════════════════════════════════════════════════════

QStringList MapProject::submapIds() const {
    return submaps_.keys();
}

SubMap* MapProject::submap(const QString& id) {
    auto it = submaps_.find(id);
    return (it != submaps_.end()) ? it.value().get() : nullptr;
}

const SubMap* MapProject::submap(const QString& id) const {
    auto it = submaps_.find(id);
    return (it != submaps_.end()) ? it.value().get() : nullptr;
}

void MapProject::addSubmap(std::shared_ptr<SubMap> sm) {
    submaps_[sm->id()] = sm;
    dirty_ = true;
}

void MapProject::removeSubmap(const QString& id) {
    submaps_.remove(id);
    dirty_ = true;
}

Route* MapProject::route(const QString& name) {
    auto it = routes_.find(name);
    return (it != routes_.end()) ? &it.value() : nullptr;
}

} // namespace me
