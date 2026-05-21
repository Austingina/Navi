#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  SubmapPanel — 左侧子地图树形面板
// ═══════════════════════════════════════════════════════════════════════

#include "core/MapProject.h"
#include <QWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QPushButton>
#include <QCheckBox>
#include <QMimeData>

namespace me {

// ── 支持语义点拖入路线的 TreeWidget ──────────────────────────────────
class DragDropTree : public QTreeWidget {
    Q_OBJECT
public:
    using QTreeWidget::QTreeWidget;

signals:
    void waypointDropped(const QString& routeName,
                         const QString& submapId,
                         const QString& pointName,
                         int insertIndex);
    void routeRefDropped(const QString& routeName,
                         const QString& nestedRouteName,
                         int insertIndex);

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

// ── SubmapPanel ──────────────────────────────────────────────────────
class SubmapPanel : public QWidget {
    Q_OBJECT
public:
    explicit SubmapPanel(QWidget* parent = nullptr);

    /// 从 MapProject 刷新整棵树
    void rebuild(MapProject* project);

    /// 展开指定路线项
    void expandRoute(const QString& routeName);

    /// 勾选后，在树中点击语义点时将以该点世界坐标为轨道相机旋转/缩放中心
    bool orbitCenterFollowsSemantic() const;

signals:
    // ── 选中事件 ─────────────────────────────────────────────────────
    void submapSelected(const QString& submapId);
    void semanticPointSelected(const QString& submapId, const QString& pointName);
    void portalSelected(const QString& submapId, int portalIndex);
    void routeSelected(const QString& routeName);

    // ── 可见性 ───────────────────────────────────────────────────────
    void submapVisibilityChanged(const QString& submapId, bool visible);
    void showSemanticPointsChanged(const QString& submapId, bool show);
    void showPortalsChanged(const QString& submapId, bool show);
    void showVprFeaturesChanged(const QString& submapId, bool show);
    void routeVisibilityChanged(const QString& routeName, bool visible);

    /// 用户在 VPR 节点上选了"删除选中 VPR 视觉特征"
    void deleteSelectedVprFeaturesRequested(const QString& submapId);

    /// 右键菜单：将视角轨道中心设到该语义点
    void orbitCenterAtSemanticRequested(const QString& submapId, const QString& pointName);

    // ── 操作请求 ─────────────────────────────────────────────────────
    void addSubmapRequested();
    void removeSubmapRequested(const QString& submapId);
    void addSemanticPointRequested(const QString& submapId);
    void addPortalRequested(const QString& submapId);
    void alignSubmapRequested(const QString& submapId);
    void buildVprDatabaseRequested(const QString& submapId);
    void viewVprDatabaseRequested(const QString& submapId);
    void addRouteRequested();
    void removeRouteRequested(const QString& routeName);
    void renameRouteRequested(const QString& oldName, const QString& newName);
    void renameSemanticPointRequested(const QString& submapId,
                                      const QString& oldName,
                                      const QString& newName);
    void setDefaultRouteRequested(const QString& routeName);
    void setDefaultSubmapRequested(const QString& submapId);
    void setParentSubmapRequested(const QString& submapId, const QString& parentId);
    void showTfTreeRequested();

    // ── 路线编辑 ─────────────────────────────────────────────────────
    void routeWaypointInserted(const QString& routeName,
                               const QString& submapId,
                               const QString& pointName,
                               int insertIndex);
    void routeRefInserted(const QString& routeName,
                          const QString& nestedRouteName,
                          int insertIndex);
    void routeWaypointRemoveRequested(const QString& routeName, int waypointIndex);

private slots:
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onContextMenu(const QPoint& pos);

private:
    void setupUI();
    QTreeWidgetItem* makeSubmapItem(const QString& id, SubMap* sm);
    QTreeWidgetItem* makeRouteItem(const QString& name, const Route& route);

    DragDropTree* tree_ = nullptr;
    QPushButton* btnAdd_ = nullptr;
    QCheckBox* orbitSemanticCenterCk_ = nullptr;

    // 记录 item 类型和数据
    enum ItemRole { RoleType = Qt::UserRole, RoleSubmapId, RolePointName, RolePortalIdx, RoleRouteName };
    enum ItemType { TypeSubmapRoot, TypeSubmap, TypeMapCloud, TypeSemPointsFolder, TypeSemPoint,
                    TypePortalsFolder, TypePortal, TypeVprFeatures,
                    TypeRoutesRoot, TypeRoute, TypeRouteWaypoint };

    MapProject* project_ = nullptr;

    friend class DragDropTree;
};

} // namespace me
