#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  MainWindow — 地图编辑器主窗口
// ═══════════════════════════════════════════════════════════════════════

#include "core/MapProject.h"
#include "core/VprDatabaseData.h"
#include "core/Scene.h"
#include "ui/PointCloudViewer.h"
#include "ui/SubmapPanel.h"
#include "ui/ScenePanel.h"
#include "ui/SceneAlignView.h"

class QStackedWidget;
class QTabWidget;
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
    void onOpenSceneAlign();

    // ── 场景：导入 / 选择 / 对齐 ─────────────────────────────────────
    void onImportOsm();
    void onImportSlamMap();
    void onNewSlamMap();   ///< 另起炉灶新建空白 SLAM 地图
    void onOpenScene();
    void onSaveScene();
    void onSlamMapSelected(const QString& slamMapId);
    void onSlamMapActivated(const QString& slamMapId);
    void onRemoveSlamMap(const QString& slamMapId);
    void onRenameSlamMap(const QString& slamMapId, const QString& newName,
                         const QString& newCode);
    void onExportSlamMap(const QString& slamMapId);
    void onAlignSlamMap(const QString& slamMapId);
    void onOrthoVisibilityChanged(const QString& slamMapId, bool visible);
    void onLeftTabChanged(int index);

    // ── 场景地图 Tab：右侧属性栏（OSM 底图 / SLAM geo_anchor）─────────
    void onOsmSelected();
    /// 在总览里高亮某张 SLAM 地图，并在右侧属性栏显示其 geo_anchor
    void showSlamGeoInPanel(const QString& slamMapId);
    /// 右侧属性栏编辑 geo_anchor → 写回场景并刷新总览
    void onSlamGeoAnchorEdited(const QString& slamMapId, const GeoAnchor& anchor);
    void onSlamLockChanged(const QString& slamMapId, bool locked);

    // ── 场景语义点位（OSM 底图上，经纬度 + 航向角）───────────────────
    void onAddScenePoint();                                   ///< 进入点选新增模式（未归属区域）
    void onAddScenePointInArea(const QString& area);          ///< 点选新增到指定区域
    void onScenePointSelected(const QString& name);           ///< 选中并在右侧编辑
    void onScenePointPlacedOnMap(double lon, double lat);     ///< 底图点选回填/新建
    void onScenePointMovedOnMap(const QString& name, double lon, double lat);
    void onSceneSemanticPointApply(const QString& oldName, const SceneSemanticPoint& pt);
    void onRemoveScenePoint(const QString& name);
    void onRenameScenePoint(const QString& oldName, const QString& newName);
    void onMoveScenePointToArea(const QString& name, const QString& area);
    void onAddScenePointToRoute(const QString& pointName, const QString& routeName,
                                int insertIndex);

    // ── 语义点位区域 ─────────────────────────────────────────────────
    void onAddScenePointArea();
    void onRenameScenePointArea(const QString& oldName, const QString& newName);
    void onRemoveScenePointArea(const QString& name);

    // ── 场景级传送门 ─────────────────────────────────────────────────
    void onAddScenePortal();
    void onEditScenePortal(const QString& name);
    void onRemoveScenePortal(const QString& name);

    // ── 全局路线 ─────────────────────────────────────────────────────
    void onAddGlobalRoute();
    void onRemoveGlobalRoute(const QString& name);
    void onRenameGlobalRoute(const QString& oldName, const QString& newName);
    void onAddGlobalWaypoint(const QString& routeName);
    void onRemoveGlobalWaypoint(const QString& routeName, int index);

    // ── 内部路线（属于某张 SLAM 地图）───────────────────────────────
    void onAddInternalRoute(const QString& mapId);
    void onRemoveInternalRoute(const QString& mapId, const QString& routeName);

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

    /// 统一保存到「场景目录」：复制 OSM/PBF 与各地图 PCD 进目录，scene.json 用相对路径。
    bool saveSceneToDir(const QString& dir);

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

    // ── 场景（分层地图：OSM + 多张 SLAM 地图 + 全局路线）─────────────
    Scene scene_;
    /// 把当前 project_ 的内容写回活跃 SLAM 地图条目（切换/保存/刷新前调用）
    void syncActiveEntryFromProject();
    /// 从活跃条目载入到 project_
    void loadProjectFromActiveEntry();
    /// 从 3D 视图移除当前活跃地图的所有显示（切换地图前调用）
    void clearViewerActiveMap();
    /// 刷新左侧两个面板（地图树 + 路线）
    void refreshScenePanels();
    /// 仅刷新场景语义点位（地图树 + 总览点位覆盖层），不重建足迹、不复位视图
    void refreshScenePointsOnly();
    /// 刷新活跃地图在 3D 中的显示（点云/标记/路线/VPR）
    void refreshActiveMapView();
    /// 在主视图（场景总览）里重建 OSM + 各 SLAM 地图正交投影
    void rebuildSceneOverview();
    /// 进入「场景地图」Tab 时确保总览已构建（惰性）
    void ensureSceneOverview();
    /// 切到某个左侧 Tab（0=场景地图 1=SLAM地图）并同步主视图
    void showLeftTab(int index);
    /// 取某张地图的可编辑 MapProject（活跃→project_；否则→条目内 project）
    MapProject* projectForMap(const QString& mapId);
    QString activeMapId() const;

    // ── 组件 ─────────────────────────────────────────────────────────
    QStackedWidget*   centralStack_ = nullptr;   ///< [0]=场景总览 [1]=3D 活跃地图
    SceneAlignView*   sceneView_ = nullptr;       ///< 场景总览：OSM + 各地图正交投影
    PointCloudViewer* viewer_ = nullptr;
    QTabWidget*       leftTabs_ = nullptr;
    ScenePanel*       scenePanel_ = nullptr;       ///< 「场景地图」Tab（含全局路线）
    SubmapPanel*      submapPanel_ = nullptr;       ///< 「SLAM地图」Tab（含活跃地图内部路线）
    PropertiesPanel*  propsPanel_ = nullptr;
    QLabel*           statusLabel_ = nullptr;
    QDoubleSpinBox*   globalVoxelSpin_ = nullptr;

    bool sceneOverviewDirty_ = true;   ///< 场景组成变化后需重建总览足迹
    QVector<QString> sceneViewRowToId_;  ///< 总览足迹行 → SLAM 地图 id

    // 场景语义点位交互状态
    QString selectedScenePoint_;      ///< 当前选中的场景语义点位名称
    QString scenePointPickForName_;   ///< 非空：底图点选用于回填该点位；空：新建
    QString pendingScenePointArea_;   ///< 新建点位时归属的区域（空=未归属）
    static constexpr int kSceneTab = 0, kSlamTab = 1;

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

    // 拖拽撤销：Ctrl+Z 分发（场景总览 → SLAM 地图对齐撤销；否则 → 语义点撤销）
    void onUndo();
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
