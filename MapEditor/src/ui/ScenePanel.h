#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  ScenePanel — 「场景地图」Tab
//    树状：OSM 底图(root) → 各 SLAM 地图
//    · 每张 SLAM 地图带「显示」勾选框，控制其正交投影是否在主视图叠加显示
//    · 双击某张 SLAM 地图 → 激活它（切到「SLAM地图」Tab，进入内部 submap 编辑）
// ═══════════════════════════════════════════════════════════════════════

#include "core/Scene.h"
#include <QWidget>
#include <QTreeWidget>

namespace me {

/// 支持把「语义点位」拖到「区域 / 全局路线」节点上的场景树。
/// 采用与「SLAM地图」路线一致的 mimeData/dropMimeData 模型：原生插入指示线、
/// 按落点位置计算插入索引，拖拽为 Copy 语义（不搬移原节点）。
class SceneTreeWidget : public QTreeWidget {
    Q_OBJECT
public:
    using QTreeWidget::QTreeWidget;

signals:
    /// 语义点位拖入全局路线（按落点位置给出插入索引）。
    void scenePointDroppedToRoute(const QString& pointName, const QString& routeName,
                                  int insertIndex);
    /// 语义点位拖到区域（area 为空表示未归属）。
    void scenePointDroppedToArea(const QString& pointName, const QString& area);

protected:
    QStringList mimeTypes() const override;
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
    QMimeData* mimeData(const QList<QTreeWidgetItem*>& items) const override;
#else
    QMimeData* mimeData(const QList<QTreeWidgetItem*> items) const override;
#endif
    bool dropMimeData(QTreeWidgetItem* parent, int index,
                      const QMimeData* data, Qt::DropAction action) override;
    Qt::DropActions supportedDropActions() const override;
};

class ScenePanel : public QWidget {
    Q_OBJECT
public:
    explicit ScenePanel(QWidget* parent = nullptr);

    void rebuild(Scene* scene);

signals:
    void importOsmRequested();
    void addSlamMapRequested();
    void removeSlamMapRequested(const QString& id);
    void renameSlamMapRequested(const QString& id, const QString& newName,
                                const QString& newCode);  ///< 右键重命名（全名 + code）
    void exportSlamMapRequested(const QString& id);           ///< 右键：单独导出为独立工程
    void slamMapActivated(const QString& id);                 ///< 双击：激活并进入编辑
    void slamMapSelected(const QString& id);                  ///< 单击：在总览里高亮
    void osmSelected();                                        ///< 单击 OSM 底图节点
    void orthoVisibilityChanged(const QString& id, bool visible);

    // ── 全局路线（跨多张 SLAM 地图 / OSM，展示在场景地图下面）───────────
    void addGlobalRouteRequested();
    void removeGlobalRouteRequested(const QString& name);
    void renameGlobalRouteRequested(const QString& oldName, const QString& newName);
    void addGlobalWaypointRequested(const QString& routeName);
    void removeGlobalWaypointRequested(const QString& routeName, int index);
    void globalRouteSelected(const QString& name);
    void globalRouteVisibilityChanged(const QString& name, bool visible);

    // ── 场景语义点位（OSM 底图上，经纬度 + 航向角）───────────────────
    void addScenePointRequested();                            ///< 请求在底图上点选新增（未归属区域）
    void addScenePointInAreaRequested(const QString& area);   ///< 在指定区域新增点位
    void scenePointSelected(const QString& name);             ///< 单击选中某个点位
    void removeScenePointRequested(const QString& name);
    void renameScenePointRequested(const QString& oldName, const QString& newName);
    void moveScenePointToAreaRequested(const QString& name, const QString& area);
    /// 将语义点位拖入某条全局路线（按落点位置插入途经点）。
    void addScenePointToRouteRequested(const QString& pointName, const QString& routeName,
                                       int insertIndex);

    // ── 语义点位区域（按区域聚合）────────────────────────────────────
    void addScenePointAreaRequested();
    void renameScenePointAreaRequested(const QString& oldName, const QString& newName);
    void removeScenePointAreaRequested(const QString& name);

    // ── 场景级传送门（语义点位 → SLAM 地图入口）──────────────────────
    void addScenePortalRequested();
    void editScenePortalRequested(const QString& name);
    void removeScenePortalRequested(const QString& name);

private slots:
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onContextMenu(const QPoint& pos);

private:
    friend class SceneTreeWidget;
    void setupUI();

    SceneTreeWidget* tree_ = nullptr;
    Scene* scene_ = nullptr;

    enum ItemRole { RoleType = Qt::UserRole, RoleId, RoleName, RoleIndex, RoleArea };
    enum ItemType { TOsm, TSlam, TGlobalRoot, TGlobalRoute, TWaypoint, TScenePoint,
                    TSpRoot, TSpArea, TPortalRoot, TScenePortal };
};

} // namespace me
