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
    /// 从目录加载: dirPath/dirName.json + PCD 文件
    bool load(const QString& dirPath);
    /// 保存到目录: dirPath/dirName.json + PCD 文件
    bool save(const QString& dirPath) const;

    // ── Submap 管理 ──────────────────────────────────────────────────
    QString defaultSubmapId() const { return defaultSubmapId_; }
    void setDefaultSubmapId(const QString& id) { defaultSubmapId_ = id; }

    QStringList submapIds() const;
    SubMap* submap(const QString& id);
    const SubMap* submap(const QString& id) const;
    void addSubmap(std::shared_ptr<SubMap> sm);
    void removeSubmap(const QString& id);
    int  submapCount() const { return static_cast<int>(submaps_.size()); }

    /// 计算 submap 到根 (default_submap) 的级联变换矩阵
    Eigen::Matrix4d submapToRootMatrix(const QString& id) const;

    /// 检查将 childId 的 parent 设为 parentId 是否会形成环
    bool wouldFormCycle(const QString& childId, const QString& parentId) const;

    /// 获取某个 submap 的直接子节点列表
    QStringList submapChildren(const QString& parentId) const;

    /// 验证 id 是否为 TF 树的根节点（无父节点且等于 defaultSubmapId）
    bool isTreeRoot(const QString& id) const;

    // ── Route 管理 ───────────────────────────────────────────────────
    QString defaultRouteId() const { return defaultRouteId_; }
    void setDefaultRouteId(const QString& name) { defaultRouteId_ = name; }

    const QMap<QString, Route>& routes() const { return routes_; }
    Route* route(const QString& name);
    void addRoute(const Route& r) { routes_[r.name] = r; }
    void removeRoute(const QString& name);
    bool renameRoute(const QString& oldName, const QString& newName);


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
    QString  defaultRouteId_;
    bool     dirty_ = false;

    // 使用 ordered map 保持插入顺序
    QMap<QString, std::shared_ptr<SubMap>> submaps_;
    QMap<QString, Route>                   routes_;
};

} // namespace me
