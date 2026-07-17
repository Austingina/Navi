#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  SubMap — 单个子地图的数据模型
// ═══════════════════════════════════════════════════════════════════════

#include "core/Types.h"
#include <QMap>
#include <memory>

namespace me {

class SubMap {
public:
    SubMap() = default;
    explicit SubMap(const QString& id);

    // ── 基本属性 ─────────────────────────────────────────────────────
    QString id()          const { return id_; }
    QString name()        const { return name_; }
    QString description() const { return description_; }
    QString mapFile()     const { return mapFile_; }

    void setId(const QString& v)          { id_ = v; }
    void setName(const QString& v)        { name_ = v; }
    void setDescription(const QString& v) { description_ = v; }
    void setMapFile(const QString& v)     { mapFile_ = v; }

    // ── 变换 (本 submap → 父 submap，根节点无父) ──────────────────────
    const RigidTransform& transform() const          { return transform_; }
    void setTransform(const RigidTransform& t)        { transform_ = t; }
    Eigen::Matrix4d transformMatrix() const           { return transform_.toMatrix(); }

    /// 父 submap ID（空字符串表示根节点 / default_submap）
    QString parentSubmapId() const                    { return parentSubmapId_; }
    void setParentSubmapId(const QString& id)         { parentSubmapId_ = id; }

    // ── 点云 ─────────────────────────────────────────────────────────
    CloudPtr cloud()        const { return cloud_; }
    CloudPtr displayCloud() const { return displayCloud_; }
    void setCloud(CloudPtr c);
    void updateDisplayCloud(float voxelSize);

    // ── 语义点位 ─────────────────────────────────────────────────────
    const QMap<QString, SemanticPoint>& semanticPoints() const { return semanticPoints_; }
    void addSemanticPoint(const SemanticPoint& sp)    { semanticPoints_[sp.name] = sp; }
    void removeSemanticPoint(const QString& name)     { semanticPoints_.remove(name); }
    bool hasSemanticPoint(const QString& name) const  { return semanticPoints_.contains(name); }
    SemanticPoint* semanticPoint(const QString& name);
    /// const 版查找：不触发隐式共享 QMap 的 detach。
    const SemanticPoint* semanticPoint(const QString& name) const;

    // ── 传送门 ───────────────────────────────────────────────────────
    const QVector<Portal>& portals() const         { return portals_; }
    QVector<Portal>&       portals()               { return portals_; }
    void addPortal(const Portal& p)                { portals_.append(p); }
    void removePortal(int index);
    void clearPortals()                            { portals_.clear(); }

    // ── 显示选项 ─────────────────────────────────────────────────────
    SubmapDisplayOptions& displayOptions()             { return displayOpts_; }
    const SubmapDisplayOptions& displayOptions() const { return displayOpts_; }

private:
    QString            id_;
    QString            name_;
    QString            description_;
    QString            mapFile_;
    RigidTransform     transform_;
    QString            parentSubmapId_;  // 空 = root (default_submap)

    CloudPtr           cloud_;           // 原始点云
    CloudPtr           displayCloud_;    // 降采样后的显示点云

    QMap<QString, SemanticPoint> semanticPoints_;
    QVector<Portal>              portals_;
    SubmapDisplayOptions         displayOpts_;
};

} // namespace me
