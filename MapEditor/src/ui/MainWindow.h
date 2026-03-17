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

    void onSubmapVisibilityChanged(const QString& id, bool visible);
    void onShowSemanticPointsChanged(const QString& id, bool show);
    void onShowPortalsChanged(const QString& id, bool show);

    void onAddSubmap();
    void onRemoveSubmap(const QString& id);
    void onAddSemanticPoint(const QString& smId);
    void onAddPortal(const QString& smId);
    void onAlignSubmap(const QString& smId);

    // ── 属性面板事件 ────────────────────────────────────────────────
    void onSubmapPropertiesChanged(const QString& id);
    void onColorChanged(const QString& id, const QColor& color);

    // ── 拾取 ─────────────────────────────────────────────────────────
    void onPointPicked(const Eigen::Vector3d& pos);

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

    /// 为新增 submap 分配一个默认颜色
    QColor nextColor();

    // ── 组件 ─────────────────────────────────────────────────────────
    PointCloudViewer* viewer_ = nullptr;
    SubmapPanel*      submapPanel_ = nullptr;
    PropertiesPanel*  propsPanel_ = nullptr;
    QLabel*           statusLabel_ = nullptr;

    MapProject project_;
    QString    currentJsonPath_;

    // 拾取回调
    enum PendingAction { NoPending, PendingAddSemanticPoint, PendingAddPortal, PendingAlignGuess };
    PendingAction pendingAction_ = NoPending;
    QString       pendingSubmapId_;

    // 颜色循环
    int colorIndex_ = 0;
    static const QVector<QColor>& defaultColors();
};

} // namespace me
