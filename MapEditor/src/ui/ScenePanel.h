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

class ScenePanel : public QWidget {
    Q_OBJECT
public:
    explicit ScenePanel(QWidget* parent = nullptr);

    void rebuild(Scene* scene);

signals:
    void importOsmRequested();
    void addSlamMapRequested();
    void removeSlamMapRequested(const QString& id);
    void renameSlamMapRequested(const QString& id, const QString& newName);  ///< 右键重命名
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
    void addScenePointRequested();                            ///< 请求在底图上点选新增
    void scenePointSelected(const QString& name);             ///< 单击选中某个点位
    void removeScenePointRequested(const QString& name);
    void renameScenePointRequested(const QString& oldName, const QString& newName);

private slots:
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onContextMenu(const QPoint& pos);

private:
    void setupUI();

    QTreeWidget* tree_ = nullptr;
    Scene* scene_ = nullptr;

    enum ItemRole { RoleType = Qt::UserRole, RoleId, RoleName, RoleIndex };
    enum ItemType { TOsm, TSlam, TGlobalRoot, TGlobalRoute, TWaypoint, TScenePoint };
};

} // namespace me
