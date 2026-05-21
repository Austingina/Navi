#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  MainWindow — 地图编辑器主窗口
// ═══════════════════════════════════════════════════════════════════════

#include "core/MapProject.h"
#include "core/VprDatabaseData.h"
#include "ui/PointCloudViewer.h"
#include "ui/SubmapPanel.h"
#include "ui/PropertiesPanel.h"
#include "ui/BuildVprDatabaseDialog.h"
#include <QMainWindow>
#include <QMap>
#include <QSet>
#include <QPair>
#include <QStatusBar>
#include <QLabel>
#include <QDoubleSpinBox>
#include <memory>

namespace me {

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    // ── 菜单 ─────────────────────────────────────────────────────────
    void onOpen();
    void onSave();
    void onSaveAs();
    void onEditVersion();
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
    void onRouteRefInserted(const QString& routeName, const QString& nestedRouteName,
                            int insertIndex);
    void onRouteWaypointRemoveRequested(const QString& routeName, int waypointIndex);

    void onAddSubmap();
    void onAddRoute();
    void onRenameRoute(const QString& oldName, const QString& newName);
    void onSetDefaultRoute(const QString& routeName);
    void onRemoveSubmap(const QString& id);
    void onAddSemanticPoint(const QString& smId);
    void onRenameSemanticPoint(const QString& smId,
                               const QString& oldName,
                               const QString& newName);
    void onAddPortal(const QString& smId);
    void onAlignSubmap(const QString& smId);
    void onBuildVprDatabase(const QString& smId);
    void onViewVprDatabase(const QString& smId);

    // ── VPR 视觉特征点（紫色点）─────────────────────────────────────
    void onShowVprFeaturesChanged(const QString& smId, bool show);
    void onVprPointClicked(const QString& smId, int index);
    void onVprSelectionChanged(const QSet<QPair<QString, int>>& selected);
    void onDeleteSelectedVprFeaturesRequested(const QString& smId);
    void onSetDefaultSubmap(const QString& id);
    void onSetParentSubmap(const QString& submapId, const QString& parentId);
    void onShowTfTree();

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

    /// 标记工程为已修改（dirty），并刷新窗口标题（追加 *）
    void markDirty();
    /// 根据当前工程路径与 dirty 状态刷新窗口标题
    void updateWindowTitle();
    /// 若有未保存更改则弹出对话框确认；返回是否允许继续（保存/丢弃 → true，取消 → false）
    bool maybePromptSaveChanges();

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

    // 记忆每个 submap 上次填的 VPR 构建参数（仅本次运行内有效）
    QMap<QString, BuildVprParams> lastBuildVprParams_;

    // VPR 数据库内存缓存（按需 load）+ 当前选中集合（由 PointCloudViewer 维护，
    // 这里只是同步一份用于"删除"操作时知道删哪些）
    QMap<QString, std::shared_ptr<VprDatabaseData>> vprDbs_;
    QSet<QPair<QString, int>>                       vprSelection_;
    /// 找出/加载某 submap 的 VPR 数据库；找不到返回 nullptr
    std::shared_ptr<VprDatabaseData> ensureVprDatabase(const QString& smId);
    /// 把内存中的 VprDatabaseData push 给 PointCloudViewer（不改 visible）
    void pushVprPointsToViewer(const QString& smId);
    /// 计算某 submap 用于 VPR 点的世界变换（与点云一致）
    Eigen::Matrix4d vprWorldTransform(const QString& smId) const;

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
