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
    void slamMapActivated(const QString& id);                 ///< 双击：激活并进入编辑
    void slamMapSelected(const QString& id);                  ///< 单击：在总览里高亮
    void orthoVisibilityChanged(const QString& id, bool visible);

private slots:
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemDoubleClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onContextMenu(const QPoint& pos);

private:
    void setupUI();

    QTreeWidget* tree_ = nullptr;
    Scene* scene_ = nullptr;

    enum ItemRole { RoleType = Qt::UserRole, RoleId };
    enum ItemType { TOsm, TSlam };
};

} // namespace me
