#include "core/MapProject.h"

#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QDateTime>
#include <QDebug>
#include <QRegularExpression>
#include <QTextStream>
#include <QCryptographicHash>

#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <QSet>

namespace me {

// ═══════════════════════════════════════════════════════════════════════
//  紧凑 JSON 格式化: position/orientation/translation/rotation 在一行
// ═══════════════════════════════════════════════════════════════════════

static QString compactifyJson(const QString& indentedJson) {
    QStringList lines = indentedJson.split('\n');
    QString result;
    QTextStream out(&result);
    
    QStringList compactKeys = {"position", "orientation", "translation", "rotation"};
    bool inCompactBlock = false;
    QString compactLine;
    
    for (int i = 0; i < lines.size(); ++i) {
        const QString& line = lines[i];
        QString trimmed = line.trimmed();
        
        // 检测是否开始一个需要压缩的块
        bool startsCompact = false;
        for (const QString& key : compactKeys) {
            if (trimmed.startsWith("\"" + key + "\"") && trimmed.contains("{")) {
                startsCompact = true;
                break;
            }
        }
        
        if (startsCompact && !trimmed.endsWith("},") && !trimmed.endsWith("}")) {
            // 开始收集压缩块
            inCompactBlock = true;
            // 保留缩进
            int indent = line.indexOf('"');
            compactLine = QString(indent, ' ') + trimmed;
        } else if (inCompactBlock) {
            // 继续收集
            if (trimmed == "}" || trimmed == "},") {
                compactLine += trimmed;
                out << compactLine << '\n';
                inCompactBlock = false;
                compactLine.clear();
            } else {
                // 去掉换行，只保留内容
                compactLine += " " + trimmed;
            }
        } else {
            out << line << '\n';
        }
    }
    
    // 移除末尾多余换行
    while (result.endsWith("\n\n")) {
        result.chop(1);
    }
    
    return result;
}

/// 将 JSON 文本中 pose/transform 分量键 (x,y,z,w) 后的**整型数值**（含 0）写为小数点后 1 位（如 0 → 0.0，5 → 5.0）。
/// 已有小数的形式（如 1.23）不会被匹配，避免破坏精度。
static QString jsonCoordsIntegersAsOneDecimal(const QString& json) {
    // 三组: 前缀 "k": 、整数值、后缀 , 或 }
    // 注意: 不能用 R"(...)" —— 模式中 `) "` 会与结束符 `)"` 混淆，导致正则截断无效
    static const QRegularExpression re(QStringLiteral(
        "(\\\"[xyzw]\\\"\\s*:\\s*)(-?\\d+)(\\s*[,\\}])"));
    if (!re.isValid()) {
        qWarning() << "jsonCoordsIntegersAsOneDecimal: invalid regex:"
                   << re.errorString();
        return json;
    }
    QString out = json;
    out.replace(re, QStringLiteral("\\1\\2.0\\3"));
    return out;
}

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
//  加载: 从目录加载 dirPath/dirName.json + PCD
// ═══════════════════════════════════════════════════════════════════════

bool MapProject::load(const QString& dirPath) {
    QDir dir(dirPath);
    if (!dir.exists()) {
        qWarning() << "目录不存在:" << dirPath;
        return false;
    }

    // JSON 文件名 = 目录名.json
    QString dirName = dir.dirName();
    QString jsonPath = dir.absoluteFilePath(dirName + ".json");

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
    projectDir_ = dir.absolutePath();

    parseRootObject(root);

    qDebug() << "地图工程加载完成. Submaps:" << submaps_.size()
             << " Routes:" << routes_.size();
    return true;
}

bool MapProject::loadFromObject(const QJsonObject& root, const QString& pcdBaseDir) {
    projectDir_ = pcdBaseDir;
    parseRootObject(root);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  解析 root 对象（global / submaps / routes）
// ═══════════════════════════════════════════════════════════════════════
void MapProject::parseRootObject(const QJsonObject& root) {
    // ── global ───────────────────────────────────────────────────────
    if (root.contains("global")) {
        auto global = root["global"].toObject();
        defaultSubmapId_ = global["default_submap"].toString("floor");
        defaultRouteId_ = global["default_route"].toString();
        // version / build_no：旧文件缺失时回到默认值
        version_ = global.value("version").toString("1.0");
        // build_no 以整数存储，但 Qt JSON 把数字按 double 反序列化，统一走 toVariant
        buildNo_ = global.value("build_no").toVariant().toLongLong();
    } else {
        version_ = "1.0";
        buildNo_ = 0;
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

            // parent_submap (TF tree edge)
            if (smObj.contains("parent_submap"))
                sm->setParentSubmapId(smObj["parent_submap"].toString());

            // transform (relative to parent_submap)
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

                    if (spData.contains("need_inspect")) {
                        sp.needInspect = spData["need_inspect"].toBool();
                    } else {
                        sp.needInspect = SemanticPoint::defaultNeedInspect(sp.type);
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
                    if (wObj.contains("route")) {
                        wp.routeRef = wObj["route"].toString();
                    } else {
                        wp.submapId      = wObj["submap"].toString();
                        wp.semanticPoint = wObj["semantic_point"].toString();
                    }
                    route.waypoints.append(wp);
                }
            }
            routes_[route.name] = route;
        }
    }

    dirty_ = false;
    // 记录加载时的内容指纹：后续保存时若内容未变则保持 build_no 不变
    contentHash_ = contentSignature();
}

// ═══════════════════════════════════════════════════════════════════════
//  内容指纹 / build_no 刷新（内容有更新才刷新时间戳）
// ═══════════════════════════════════════════════════════════════════════
QByteArray MapProject::contentSignature() const {
    QJsonObject root = toObject();
    // 剔除 build_no 本身，避免时间戳参与指纹计算造成自我循环
    QJsonObject g = root.value("global").toObject();
    g.remove("build_no");
    root["global"] = g;
    // _comment 为固定注释，不影响内容
    root.remove("_comment");
    const QByteArray bytes = QJsonDocument(root).toJson(QJsonDocument::Compact);
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha1);
}

qint64 MapProject::refreshBuildNoIfChanged() {
    const QByteArray sig = contentSignature();
    if (buildNo_ == 0 || sig != contentHash_) {
        buildNo_ = QDateTime::currentDateTime()
                       .toString(QStringLiteral("yyyyMMddhhmmss"))
                       .toLongLong();
        contentHash_ = sig;
    }
    return buildNo_;
}

// ═══════════════════════════════════════════════════════════════════════
//  保存: 到目录 dirPath/dirName.json + PCD
// ═══════════════════════════════════════════════════════════════════════

bool MapProject::save(const QString& dirPath) {
    QDir dir(dirPath);
    if (!dir.exists()) {
        QDir().mkpath(dirPath);
    }

    // JSON 文件名 = 目录名.json
    QString dirName = dir.dirName();
    QString jsonPath = dir.absoluteFilePath(dirName + ".json");

    // build_no 采用 YYYYMMDDHHMMSS（本地时间，14 位整数）：
    // - 既单调递增（供 mapserver 比对"是否有新版本"），又人类可读
    // - 14 位最大 ~1e14 ≪ 2^63，qint64 范围充裕
    // 仅在「内容较上次加载/保存有更新」时才刷新时间戳，否则保持不变。
    refreshBuildNoIfChanged();

    QJsonObject root = toObject();

    // ── 写文件 ───────────────────────────────────────────────────────
    QFile file(jsonPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "无法写入文件:" << jsonPath;
        return false;
    }

    QJsonDocument doc(root);
    QString indented = QString::fromUtf8(doc.toJson(QJsonDocument::Indented));
    QString compact = compactifyJson(indented);
    compact = jsonCoordsIntegersAsOneDecimal(compact);

    QTextStream out(&file);
    out.setCodec("UTF-8");
    out << compact;
    file.close();

    // 保存各 submap 的 PCD (原始精度)
    savePcds(dir.absolutePath());

    qDebug() << "地图工程已保存至:" << jsonPath;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  JSON 对象导出 (供文件保存与场景 single-file 内联复用)
// ═══════════════════════════════════════════════════════════════════════
QJsonObject MapProject::toObject() const {
    QJsonObject root;
    root["_comment"] = QString("多楼层地图配置文件 / Multi-floor map configuration");

    QString savedDefaultRoute = defaultRouteId_;
    if (savedDefaultRoute.isEmpty() || !routes_.contains(savedDefaultRoute)) {
        if (!routes_.isEmpty())
            savedDefaultRoute = routes_.firstKey();
    }
    QJsonObject globalObj;
    globalObj["default_submap"] = defaultSubmapId_;
    globalObj["default_route"]  = savedDefaultRoute;
    globalObj["version"]        = version_;
    globalObj["build_no"]       = QJsonValue(static_cast<qint64>(buildNo_));
    root["global"] = globalObj;

    QJsonObject submapsObj;
    for (auto it = submaps_.begin(); it != submaps_.end(); ++it) {
        const auto& sm = it.value();
        QJsonObject smObj;
        smObj["name"]        = sm->name();
        smObj["description"] = sm->description();
        smObj["map_file"]    = sm->mapFile();
        if (!sm->parentSubmapId().isEmpty())
            smObj["parent_submap"] = sm->parentSubmapId();
        smObj["transform"]   = transformToJson(sm->transform());

        QJsonArray portalsArr;
        for (const auto& p : sm->portals()) {
            portalsArr.append(QJsonObject{
                {"portal", p.portalName}, {"submap", p.targetSubmap}});
        }
        smObj["submap_portals"] = portalsArr;

        QJsonObject spObj;
        for (auto sit = sm->semanticPoints().begin();
             sit != sm->semanticPoints().end(); ++sit) {
            const auto& sp = sit.value();
            QJsonObject spData;
            spData["type"] = sp.type;
            if (sp.type == "multi_pose") {
                QJsonArray posesArr;
                for (const auto& pose : sp.poses) posesArr.append(poseToJson(pose));
                spData["poses"] = posesArr;
            } else if (!sp.poses.isEmpty()) {
                spData["pose"] = poseToJson(sp.poses.first());
            }
            spData["need_inspect"] = sp.needInspect;
            spObj[sp.name] = spData;
        }
        smObj["semantic_points"] = spObj;
        submapsObj[it.key()] = smObj;
    }
    root["submaps"] = submapsObj;

    QJsonObject routesObj;
    for (auto it = routes_.begin(); it != routes_.end(); ++it) {
        const auto& r = it.value();
        QJsonObject rObj;
        rObj["description"] = r.description;
        QJsonArray wpArr;
        for (const auto& wp : r.waypoints) {
            if (wp.isRouteRef()) {
                wpArr.append(QJsonObject{{"route", wp.routeRef}});
            } else {
                wpArr.append(QJsonObject{
                    {"submap", wp.submapId}, {"semantic_point", wp.semanticPoint}});
            }
        }
        rObj["waypoints"] = wpArr;
        routesObj[it.key()] = rObj;
    }
    root["routes"] = routesObj;
    return root;
}

bool MapProject::savePcds(const QString& pcdBaseDir) const {
    QDir dir(pcdBaseDir);
    if (!dir.exists()) QDir().mkpath(pcdBaseDir);
    bool ok = true;
    for (auto it = submaps_.begin(); it != submaps_.end(); ++it) {
        const auto& sm = it.value();
        if (sm->cloud() && !sm->mapFile().isEmpty()) {
            QString pcdPath = dir.absoluteFilePath(sm->mapFile());
            QFileInfo(pcdPath).absoluteDir().mkpath(".");
            if (!savePcd(pcdPath, sm->cloud())) ok = false;
        }
    }
    return ok;
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

Eigen::Matrix4d MapProject::submapToRootMatrix(const QString& id) const {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    QSet<QString> visited;
    QString cur = id;
    while (!cur.isEmpty()) {
        if (visited.contains(cur)) break;  // cycle guard
        visited.insert(cur);
        auto it = submaps_.find(cur);
        if (it == submaps_.end()) break;
        result = it.value()->transformMatrix() * result;
        cur = it.value()->parentSubmapId();
    }
    return result;
}

bool MapProject::wouldFormCycle(const QString& childId, const QString& parentId) const {
    if (childId == parentId) return true;
    QSet<QString> visited;
    visited.insert(childId);
    QString cur = parentId;
    while (!cur.isEmpty()) {
        if (visited.contains(cur)) return true;
        visited.insert(cur);
        auto it = submaps_.find(cur);
        if (it == submaps_.end()) break;
        cur = it.value()->parentSubmapId();
    }
    return false;
}

QStringList MapProject::submapChildren(const QString& parentId) const {
    QStringList children;
    for (auto it = submaps_.begin(); it != submaps_.end(); ++it) {
        if (it.value()->parentSubmapId() == parentId && it.key() != parentId)
            children.append(it.key());
    }
    return children;
}

bool MapProject::isTreeRoot(const QString& id) const {
    auto it = submaps_.find(id);
    if (it == submaps_.end()) return false;
    return it.value()->parentSubmapId().isEmpty();
}

Route* MapProject::route(const QString& name) {
    auto it = routes_.find(name);
    return (it != routes_.end()) ? &it.value() : nullptr;
}

const Route* MapProject::route(const QString& name) const {
    auto it = routes_.constFind(name);
    return (it != routes_.constEnd()) ? &it.value() : nullptr;
}

void MapProject::removeRoute(const QString& name) {
    routes_.remove(name);
    for (auto& route : routes_) {
        route.waypoints.erase(
            std::remove_if(route.waypoints.begin(), route.waypoints.end(),
                           [&](const RouteWaypoint& wp) { return wp.routeRef == name; }),
            route.waypoints.end());
    }
}

bool MapProject::renameSemanticPoint(const QString& submapId,
                                     const QString& oldName,
                                     const QString& newName) {
    if (newName.isEmpty() || oldName == newName) return false;
    auto* sm = submap(submapId);
    if (!sm) return false;
    if (!sm->hasSemanticPoint(oldName)) return false;
    if (sm->hasSemanticPoint(newName)) return false;

    SemanticPoint sp = sm->semanticPoints().value(oldName);
    sp.name = newName;
    sm->removeSemanticPoint(oldName);
    sm->addSemanticPoint(sp);

    // 同步更新所有引用该点的路线 waypoint
    for (auto& route : routes_) {
        for (auto& wp : route.waypoints) {
            if (wp.isRouteRef()) continue;
            if (wp.submapId == submapId && wp.semanticPoint == oldName) {
                wp.semanticPoint = newName;
            }
        }
    }

    // 如果该点同时出现在 portals（portalName 即对应语义点）里，也一并更新
    for (auto& p : sm->portals()) {
        if (p.portalName == oldName) p.portalName = newName;
    }
    return true;
}

bool MapProject::renameRoute(const QString& oldName, const QString& newName) {
    auto it = routes_.find(oldName);
    if (it == routes_.end()) return false;
    if (routes_.contains(newName)) return false;
    Route r = it.value();
    r.name = newName;
    routes_.remove(oldName);
    routes_[newName] = r;
    if (defaultRouteId_ == oldName)
        defaultRouteId_ = newName;
    for (auto& route : routes_) {
        for (auto& wp : route.waypoints) {
            if (wp.routeRef == oldName)
                wp.routeRef = newName;
        }
    }
    return true;
}

} // namespace me
