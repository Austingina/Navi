#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  MapProject — 多 Submap 地图工程：加载 / 保存 / 管理
// ═══════════════════════════════════════════════════════════════════════

#include "core/SubMap.h"
#include "core/Types.h"
#include <QMap>
#include <QString>
#include <QJsonObject>
#include <memory>

namespace me {

class MapProject {
public:
    MapProject() = default;

    // ── 加载 / 保存 ─────────────────────────────────────────────────
    /// 从 multifloor.json 加载 (pcd 相对于 json 同目录)
    bool load(const QString& jsonPath);
    /// 保存为 multifloor.json + 各 submap 的 pcd
    bool save(const QString& jsonPath) const;

    // ── Submap 管理 ──────────────────────────────────────────────────
    QString defaultSubmapId() const { return defaultSubmapId_; }
    void setDefaultSubmapId(const QString& id) { defaultSubmapId_ = id; }

    QStringList submapIds() const;
    SubMap* submap(const QString& id);
    const SubMap* submap(const QString& id) const;
    void addSubmap(std::shared_ptr<SubMap> sm);
    void removeSubmap(const QString& id);
    int  submapCount() const { return static_cast<int>(submaps_.size()); }

    // ── Route 管理 ───────────────────────────────────────────────────
    const QMap<QString, Route>& routes() const { return routes_; }
    Route* route(const QString& name);
    void addRoute(const Route& r) { routes_[r.name] = r; }
    void removeRoute(const QString& name) { routes_.remove(name); }

    // ── 项目路径 ─────────────────────────────────────────────────────
    QString projectDir() const { return projectDir_; }
    bool isDirty() const { return dirty_; }
    void setDirty(bool d = true) { dirty_ = d; }

private:
    // JSON 解析助手
    Pose           parsePose(const QJsonObject& obj) const;
    QJsonObject    poseToJson(const Pose& p) const;
    RigidTransform parseTransform(const QJsonObject& obj) const;
    QJsonObject    transformToJson(const RigidTransform& t) const;

    // 加载 PCD 文件
    CloudPtr loadPcd(const QString& path) const;
    bool     savePcd(const QString& path, CloudPtr cloud) const;

    QString  projectDir_;
    QString  defaultSubmapId_;
    bool     dirty_ = false;

    // 使用 ordered map 保持插入顺序
    QMap<QString, std::shared_ptr<SubMap>> submaps_;
    QMap<QString, Route>                   routes_;
};

} // namespace me
