#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  MainWindow — 地图编辑器主窗口
// ═══════════════════════════════════════════════════════════════════════

#include "core/MapProject.h"
#include "ui/PointCloudViewer.h"
#include "ui/SubmapPanel.h"
#include "ui/PropertiesPanel.h"
#include <QMainWindow>
#include <QStatusBar>
#include <QLabel>
#include <QDoubleSpinBox>

namespace me {

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private slots:
    // ── 菜单 ─────────────────────────────────────────────────────────
    void onOpen();
    void onSave();
    void onSaveAs();
    void onResetView();

    // ── Submap 面板事件 ──────────────────────────────────────────────
    void onSubmapSelected(const QString& id);
    void onSemanticPointSelected(const QString& smId, const QString& ptName);
    void onPortalSelected(const QString& smId, int portalIdx);
    void onPortalPropertiesChanged(const QString& smId, int portalIdx);
    void onSemanticPointChanged(const QString& smId, const QString& ptName);

    void onSubmapVisibilityChanged(const QString& id, bool visible);
    void onShowSemanticPointsChanged(const QString& id, bool show);
    void onShowPortalsChanged(const QString& id, bool show);
    void onRouteVisibilityChanged(const QString& routeName, bool visible);
    void onRouteWaypointInserted(const QString& routeName, const QString& submapId,
                                 const QString& pointName, int insertIndex);
    void onRouteWaypointRemoveRequested(const QString& routeName, int waypointIndex);

    void onAddSubmap();
    void onAddRoute();
    void onRenameRoute(const QString& oldName, const QString& newName);
    void onRemoveSubmap(const QString& id);
    void onAddSemanticPoint(const QString& smId);
    void onAddPortal(const QString& smId);
    void onAlignSubmap(const QString& smId);
    void onSetDefaultSubmap(const QString& id);

    // ── 属性面板事件 ────────────────────────────────────────────────
    void onSubmapPropertiesChanged(const QString& id);
    void onColorChanged(const QString& id, const QColor& color);

    // ── 全局降采样 ──────────────────────────────────────────────────
    void onGlobalVoxelSizeChanged(double size);

    // ── 拾取 ─────────────────────────────────────────────────────────
    void onPointPicked(const Eigen::Vector3d& pos);
    void onMarkerDragFinished(const QString& submapId, const QString& name, const Eigen::Vector3d& newPos);
    void onSemanticPickPositionRequested(const QString& smId);

private:
    void setupUI();
    void setupMenuBar();
    void connectSignals();

    /// 刷新所有 submap 的 3D 显示
    void refreshAllSubmaps();
    /// 刷新单个 submap 的 3D 显示
    void refreshSubmap(const QString& id);
    /// 刷新标记点 (语义点+传送门)
    void refreshMarkers();

    /// 将轨道相机 target 设到语义点在世界系下的位置（multi_pose 取位姿均值）
    void setViewerOrbitCenterAtSemantic(const QString& smId, const QString& ptName);
    /// 刷新所有路线折线
    void refreshRouteLines();

    /// 为新增 submap 分配一个默认颜色
    QColor nextColor();

    // ── 组件 ─────────────────────────────────────────────────────────
    PointCloudViewer* viewer_ = nullptr;
    SubmapPanel*      submapPanel_ = nullptr;
    PropertiesPanel*  propsPanel_ = nullptr;
    QLabel*           statusLabel_ = nullptr;
    QDoubleSpinBox*   globalVoxelSpin_ = nullptr;

    MapProject project_;
    QString    currentProjectDir_;

    /// 子图树中选中的语义点，用于 3D 中高亮（红色）
    QString    selectedSemanticSmId_;
    QString    selectedSemanticPtName_;

    // 拾取回调
    enum PendingAction { NoPending, PendingSemanticPanelPick,
                         PendingAlignGuess, PendingAlignSourceCenter };
    PendingAction pendingAction_ = NoPending;
    QString       pendingSubmapId_;

    // 拖拽撤销（最多一次）
    void undoMarkerDrag();
    struct MarkerDragUndo {
        bool valid = false;
        QString submapId;
        QString pointName;
        Eigen::Vector3d oldPosition{0, 0, 0};
    } markerDragUndo_;

    static QVector<MarkerVertex> buildWireframeBox(
        const Eigen::Vector3d& center, const Eigen::Vector3d& halfExtents,
        const QColor& color, const Eigen::Matrix4d& transform = Eigen::Matrix4d::Identity());

    // 颜色循环
    int colorIndex_ = 0;
    static const QVector<QColor>& defaultColors();
};

} // namespace me
