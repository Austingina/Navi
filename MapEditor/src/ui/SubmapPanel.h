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

namespace me {

class SubmapPanel : public QWidget {
    Q_OBJECT
public:
    explicit SubmapPanel(QWidget* parent = nullptr);

    /// 从 MapProject 刷新整棵树
    void rebuild(MapProject* project);

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
    void routeVisibilityChanged(const QString& routeName, bool visible);

    // ── 操作请求 ─────────────────────────────────────────────────────
    void addSubmapRequested();
    void removeSubmapRequested(const QString& submapId);
    void addSemanticPointRequested(const QString& submapId);
    void addPortalRequested(const QString& submapId);
    void alignSubmapRequested(const QString& submapId);
    void addRouteRequested();
    void removeRouteRequested(const QString& routeName);
    // 将某个 submap 设置为默认 submap
    void setDefaultSubmapRequested(const QString& submapId);

private slots:
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onContextMenu(const QPoint& pos);

private:
    void setupUI();
    QTreeWidgetItem* makeSubmapItem(const QString& id, SubMap* sm);
    QTreeWidgetItem* makeRouteItem(const QString& name, const Route& route);

    QTreeWidget* tree_ = nullptr;
    QPushButton* btnAdd_ = nullptr;

    // 记录 item 类型和数据
    enum ItemRole { RoleType = Qt::UserRole, RoleSubmapId, RolePointName, RolePortalIdx, RoleRouteName };
    enum ItemType { TypeSubmapRoot, TypeSubmap, TypeSemPointsFolder, TypeSemPoint,
                    TypePortalsFolder, TypePortal, TypeRoutesRoot, TypeRoute, TypeRouteWaypoint };

    MapProject* project_ = nullptr;
};

} // namespace me
