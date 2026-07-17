#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  RoutePanel — 路线面板
//    · 全局路线：跨多张 SLAM 地图 / OSM 的路线（Scene.globalRoutes）
//    · 内部路线：各 SLAM 地图自身的路线（MapProject.routes），按地图名分组
// ═══════════════════════════════════════════════════════════════════════

#include "core/Scene.h"
#include <QWidget>
#include <QTreeWidget>

class QPushButton;

namespace me {

class RoutePanel : public QWidget {
    Q_OBJECT
public:
    explicit RoutePanel(QWidget* parent = nullptr);

    void rebuild(Scene* scene);

signals:
    // 全局路线
    void addGlobalRouteRequested();
    void removeGlobalRouteRequested(const QString& name);
    void renameGlobalRouteRequested(const QString& oldName, const QString& newName);
    void addGlobalWaypointRequested(const QString& routeName);
    void removeGlobalWaypointRequested(const QString& routeName, int index);
    void globalRouteSelected(const QString& name);
    void globalRouteVisibilityChanged(const QString& name, bool visible);

    // 内部路线（属于某张 SLAM 地图）
    void addInternalRouteRequested(const QString& mapId);
    void removeInternalRouteRequested(const QString& mapId, const QString& routeName);
    void internalRouteSelected(const QString& mapId, const QString& routeName);
    void internalRouteVisibilityChanged(const QString& mapId, const QString& routeName, bool visible);

private slots:
    void onContextMenu(const QPoint& pos);
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);

private:
    void setupUI();

    QTreeWidget* tree_ = nullptr;
    QPushButton* btnAddGlobal_ = nullptr;
    Scene* scene_ = nullptr;

    enum ItemRole { RoleType = Qt::UserRole, RoleName, RoleMapId, RoleIndex };
    enum ItemType { TGlobalRoot, TGlobalRoute, TWaypoint,
                    TInternalRoot, TMapGroup, TInternalRoute };
};

} // namespace me
