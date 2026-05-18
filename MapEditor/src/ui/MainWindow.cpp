#include "ui/MainWindow.h"
#include "ui/AddSubmapDialog.h"
#include "ui/AlignSubmapDialog.h"
#include "ui/BuildVprDatabaseDialog.h"
#include "ui/PortalEditorDialog.h"
#include "ui/VprDatabaseInfoDialog.h"

#include <QMenuBar>
#include <QToolBar>
#include <QDockWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QSplitter>
#include <QApplication>
#include <QDialog>
#include <QPushButton>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>

#include <Eigen/Geometry>
#include <QSet>
#include <functional>

namespace me {

namespace {

/// 在 lines 中追加一段 GL_LINES 箭头：主线 + 箭头两翼；forwardWorld 为指向方向（不必已单位化）
void appendSemanticArrowLines(QVector<MarkerVertex>&        lines,
                              const Eigen::Vector3d&        baseWorld,
                              const Eigen::Vector3d&        forwardWorld,
                              float                         totalLen,
                              float                         headLen,
                              float                         headWidth,
                              float                         r,
                              float                         g,
                              float                         b) {
    Eigen::Vector3d fwd = forwardWorld;
    const double    n   = fwd.norm();
    if (n < 1e-9) return;
    fwd /= n;
    Eigen::Vector3d worldUp(0, 0, 1);
    Eigen::Vector3d side = fwd.cross(worldUp);
    if (side.norm() < 1e-6) side = fwd.cross(Eigen::Vector3d(0, 1, 0));
    side.normalize();

    const Eigen::Vector3d tip  = baseWorld + totalLen * fwd;
    const Eigen::Vector3d neck = baseWorld + (totalLen - headLen) * fwd;

    auto seg = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
        lines.append(MarkerVertex{(float)p0.x(), (float)p0.y(), (float)p0.z(), r, g, b});
        lines.append(MarkerVertex{(float)p1.x(), (float)p1.y(), (float)p1.z(), r, g, b});
    };
    seg(baseWorld, neck);
    const Eigen::Vector3d wingA = tip - headLen * fwd + headWidth * side;
    const Eigen::Vector3d wingB = tip - headLen * fwd - headWidth * side;
    seg(tip, wingA);
    seg(tip, wingB);
}

} // namespace

// ═══════════════════════════════════════════════════════════════════════
//  默认颜色表
// ═══════════════════════════════════════════════════════════════════════

const QVector<QColor>& MainWindow::defaultColors() {
    static QVector<QColor> colors = {
        QColor(100, 180, 255),   // 浅蓝
        QColor(255, 130, 80),    // 橘色
        QColor(120, 220, 120),   // 浅绿
        QColor(220, 130, 220),   // 粉紫
        QColor(255, 220, 80),    // 黄色
        QColor(80, 220, 210),    // 青色
        QColor(255, 100, 100),   // 浅红
        QColor(180, 180, 180),   // 灰色
    };
    return colors;
}

QColor MainWindow::nextColor() {
    const auto& colors = defaultColors();
    QColor c = colors[colorIndex_ % colors.size()];
    colorIndex_++;
    return c;
}

// ═══════════════════════════════════════════════════════════════════════
//  构造
// ═══════════════════════════════════════════════════════════════════════

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    resize(1400, 900);
    setupUI();
    setupMenuBar();
    connectSignals();
    updateWindowTitle();
}

// ═══════════════════════════════════════════════════════════════════════
//  Dirty / 标题 / 关闭确认
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::updateWindowTitle() {
    QString base = "MapEditor — 地图编辑器";
    if (!currentProjectDir_.isEmpty()) {
        base = QString("MapEditor — %1").arg(currentProjectDir_);
    }
    if (project_.isDirty()) base += " *";
    setWindowTitle(base);
    setWindowModified(project_.isDirty());
}

void MainWindow::markDirty() {
    project_.setDirty(true);
    updateWindowTitle();
}

bool MainWindow::maybePromptSaveChanges() {
    if (!project_.isDirty()) return true;

    auto ret = QMessageBox::question(
        this, "保存修改？",
        "当前地图存在未保存的修改，是否保存？",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
        QMessageBox::Save);

    if (ret == QMessageBox::Cancel) return false;
    if (ret == QMessageBox::Discard) return true;

    onSave();
    return !project_.isDirty();
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (maybePromptSaveChanges()) {
        event->accept();
    } else {
        event->ignore();
    }
}

void MainWindow::setupUI() {
    // ── 中心: 3D Viewer ──────────────────────────────────────────────
    viewer_ = new PointCloudViewer();
    setCentralWidget(viewer_);

    // ── 左侧: Submap 面板 ───────────────────────────────────────────
    submapPanel_ = new SubmapPanel();
    auto* leftDock = new QDockWidget("子地图管理", this);
    leftDock->setWidget(submapPanel_);
    leftDock->setMinimumWidth(260);
    addDockWidget(Qt::LeftDockWidgetArea, leftDock);

    // ── 右侧: 属性面板 ──────────────────────────────────────────────
    propsPanel_ = new PropertiesPanel();
    auto* rightDock = new QDockWidget("属性编辑", this);
    rightDock->setWidget(propsPanel_);
    rightDock->setMinimumWidth(280);
    addDockWidget(Qt::RightDockWidgetArea, rightDock);

    // ── 状态栏 ───────────────────────────────────────────────────────
    statusLabel_ = new QLabel("就绪");
    statusBar()->addWidget(statusLabel_);
}

void MainWindow::setupMenuBar() {
    auto* fileMenu = menuBar()->addMenu("文件(&F)");
    fileMenu->addAction("打开 multifloor.json...", this, &MainWindow::onOpen, QKeySequence::Open);
    fileMenu->addAction("保存", this, &MainWindow::onSave, QKeySequence::Save);
    fileMenu->addAction("另存为...", this, &MainWindow::onSaveAs, QKeySequence("Ctrl+Shift+S"));
    fileMenu->addSeparator();
    fileMenu->addAction("修改版本号...", this, &MainWindow::onEditVersion);
    fileMenu->addSeparator();
    fileMenu->addAction("退出", this, &QMainWindow::close, QKeySequence::Quit);

    auto* viewMenu = menuBar()->addMenu("视图(&V)");
    viewMenu->addAction("重置视角", this, &MainWindow::onResetView, QKeySequence("R"));

    auto* editMenu = menuBar()->addMenu("编辑(&E)");
    editMenu->addAction("撤销拖拽", this, &MainWindow::undoMarkerDrag, QKeySequence::Undo);
    editMenu->addSeparator();
    editMenu->addAction("新增 Submap...", this, &MainWindow::onAddSubmap, QKeySequence("Ctrl+N"));

    // 工具栏
    auto* toolbar = addToolBar("主工具栏");
    toolbar->addAction("📂 打开", this, &MainWindow::onOpen);
    toolbar->addAction("💾 保存", this, &MainWindow::onSave);
    toolbar->addSeparator();
    toolbar->addAction("+ Submap", this, &MainWindow::onAddSubmap);
    toolbar->addSeparator();
    toolbar->addAction("🔄 重置视角", this, &MainWindow::onResetView);

    // 全局降采样设置
    toolbar->addSeparator();
    toolbar->addWidget(new QLabel(" 降采样(m): "));
    globalVoxelSpin_ = new QDoubleSpinBox();
    globalVoxelSpin_->setRange(0.01, 5.0);
    globalVoxelSpin_->setDecimals(2);
    globalVoxelSpin_->setSingleStep(0.1);
    globalVoxelSpin_->setValue(1.0);
    globalVoxelSpin_->setToolTip("全局显示降采样体素大小 (仅影响显示, 不影响保存)");
    globalVoxelSpin_->setFixedWidth(80);
    toolbar->addWidget(globalVoxelSpin_);
    auto* btnApplyVoxel = new QPushButton("应用");
    btnApplyVoxel->setFixedWidth(50);
    toolbar->addWidget(btnApplyVoxel);
    connect(btnApplyVoxel, &QPushButton::clicked, [this]() {
        onGlobalVoxelSizeChanged(globalVoxelSpin_->value());
    });
}

void MainWindow::connectSignals() {
    // ── Submap Panel ─────────────────────────────────────────────────
    connect(submapPanel_, &SubmapPanel::submapSelected,
            this, &MainWindow::onSubmapSelected);
    connect(submapPanel_, &SubmapPanel::semanticPointSelected,
            this, &MainWindow::onSemanticPointSelected);
    connect(submapPanel_, &SubmapPanel::orbitCenterAtSemanticRequested,
            this, [this](const QString& smId, const QString& ptName) {
        setViewerOrbitCenterAtSemantic(smId, ptName);
    });
    connect(submapPanel_, &SubmapPanel::portalSelected,
            this, &MainWindow::onPortalSelected);
    connect(submapPanel_, &SubmapPanel::routeSelected, this, [this](const QString&) {
        selectedSemanticSmId_.clear();
        selectedSemanticPtName_.clear();
        refreshMarkers();
    });

    connect(submapPanel_, &SubmapPanel::submapVisibilityChanged,
            this, &MainWindow::onSubmapVisibilityChanged);
    connect(submapPanel_, &SubmapPanel::showSemanticPointsChanged,
            this, &MainWindow::onShowSemanticPointsChanged);
    connect(submapPanel_, &SubmapPanel::showPortalsChanged,
            this, &MainWindow::onShowPortalsChanged);
    connect(submapPanel_, &SubmapPanel::routeVisibilityChanged,
            this, &MainWindow::onRouteVisibilityChanged);
    connect(submapPanel_, &SubmapPanel::routeWaypointInserted,
            this, &MainWindow::onRouteWaypointInserted);
    connect(submapPanel_, &SubmapPanel::routeRefInserted,
            this, &MainWindow::onRouteRefInserted);
    connect(submapPanel_, &SubmapPanel::routeWaypointRemoveRequested,
            this, &MainWindow::onRouteWaypointRemoveRequested);

    connect(submapPanel_, &SubmapPanel::addSubmapRequested,
            this, &MainWindow::onAddSubmap);
        connect(submapPanel_, &SubmapPanel::addRouteRequested,
            this, &MainWindow::onAddRoute);
    connect(submapPanel_, &SubmapPanel::renameRouteRequested,
            this, &MainWindow::onRenameRoute);
    connect(submapPanel_, &SubmapPanel::setDefaultRouteRequested,
            this, &MainWindow::onSetDefaultRoute);
    connect(submapPanel_, &SubmapPanel::removeSubmapRequested,
            this, &MainWindow::onRemoveSubmap);
    connect(submapPanel_, &SubmapPanel::addSemanticPointRequested,
            this, &MainWindow::onAddSemanticPoint);
    connect(submapPanel_, &SubmapPanel::renameSemanticPointRequested,
            this, &MainWindow::onRenameSemanticPoint);
    connect(submapPanel_, &SubmapPanel::addPortalRequested,
            this, &MainWindow::onAddPortal);
    connect(submapPanel_, &SubmapPanel::alignSubmapRequested,
            this, &MainWindow::onAlignSubmap);
    connect(submapPanel_, &SubmapPanel::buildVprDatabaseRequested,
            this, &MainWindow::onBuildVprDatabase);
    connect(submapPanel_, &SubmapPanel::viewVprDatabaseRequested,
            this, &MainWindow::onViewVprDatabase);
        connect(submapPanel_, &SubmapPanel::setDefaultSubmapRequested,
            this, &MainWindow::onSetDefaultSubmap);
    connect(submapPanel_, &SubmapPanel::setParentSubmapRequested,
            this, &MainWindow::onSetParentSubmap);
    connect(submapPanel_, &SubmapPanel::showTfTreeRequested,
            this, &MainWindow::onShowTfTree);

    // ── Properties Panel ─────────────────────────────────────────────
    connect(propsPanel_, &PropertiesPanel::submapPropertiesChanged,
            this, &MainWindow::onSubmapPropertiesChanged);
    connect(propsPanel_, &PropertiesPanel::colorChanged,
            this, &MainWindow::onColorChanged);
        connect(propsPanel_, &PropertiesPanel::semanticPointChanged,
            this, &MainWindow::onSemanticPointChanged);
    connect(propsPanel_, &PropertiesPanel::portalChanged,
            this, &MainWindow::onPortalPropertiesChanged);
    connect(propsPanel_, &PropertiesPanel::pickSemanticPositionRequested,
            this, &MainWindow::onSemanticPickPositionRequested);

    // ── Viewer ───────────────────────────────────────────────────────
    connect(viewer_, &PointCloudViewer::pointPicked,
            this, &MainWindow::onPointPicked);
        connect(viewer_, &PointCloudViewer::markerDragFinished,
            this, &MainWindow::onMarkerDragFinished);
}

// ═══════════════════════════════════════════════════════════════════════
//  文件操作
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onOpen() {
    if (!maybePromptSaveChanges()) return;

    QString path = QFileDialog::getExistingDirectory(
        this, "选择地图目录", "",
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (path.isEmpty()) return;

    statusLabel_->setText("加载中...");
    QApplication::processEvents();

    if (!project_.load(path)) {
        QMessageBox::critical(this, "错误", "无法加载: " + path);
        statusLabel_->setText("加载失败");
        return;
    }

    currentProjectDir_ = path;
    colorIndex_ = 0;
    selectedSemanticSmId_.clear();
    selectedSemanticPtName_.clear();

    // 为每个 submap 分配颜色
    for (const auto& id : project_.submapIds()) {
        auto* sm = project_.submap(id);
        if (sm) {
            sm->displayOptions().color = nextColor();
        }
    }

    submapPanel_->rebuild(&project_);
    refreshAllSubmaps();
    refreshMarkers();
    refreshRouteLines();
    viewer_->resetCamera();

    statusLabel_->setText(QString("已加载: %1 | %2 个子地图 | %3 条路线")
                              .arg(path)
                              .arg(project_.submapCount())
                              .arg(project_.routes().size()));
    project_.setDirty(false);
    updateWindowTitle();
}

void MainWindow::onSave() {
    if (currentProjectDir_.isEmpty()) {
        onSaveAs();
        return;
    }
    // 检查是否存在 default_submap，若不存在则提示并阻止保存
    QString defId = project_.defaultSubmapId();
    if (defId.isEmpty() || !project_.submap(defId)) {
        QMessageBox::warning(this, "提示",
                             "当前项目缺少 default_submap，保存前请设置默认子地图。");
        return;
    }
    if (project_.save(currentProjectDir_)) {
        project_.setDirty(false);
        updateWindowTitle();
        statusLabel_->setText("已保存: " + currentProjectDir_);
    } else {
        QMessageBox::critical(this, "错误", "保存失败");
    }
}

void MainWindow::onSaveAs() {
    QString path = QFileDialog::getExistingDirectory(
        this, "选择保存目录", currentProjectDir_,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (path.isEmpty()) return;
    currentProjectDir_ = path;
    updateWindowTitle();
    onSave();
}

void MainWindow::onEditVersion() {
    const QString cur = project_.version();
    const qint64  bn  = project_.buildNo();

    // build_no 当前格式是 YYYYMMDDHHMMSS（14 位整数）。
    // 兼容一下早期可能写过的 epoch 秒（10 位），便于打开历史文件时不显示得太突兀。
    QString bnText;
    if (bn <= 0) {
        bnText = QStringLiteral("尚未保存");
    } else {
        const QString digits = QString::number(bn);
        QDateTime dt;
        if (digits.length() == 14) {
            dt = QDateTime::fromString(digits, QStringLiteral("yyyyMMddhhmmss"));
        } else if (bn >= 1'000'000'000LL && bn < 10'000'000'000LL) {
            dt = QDateTime::fromSecsSinceEpoch(bn);
        }
        bnText = dt.isValid()
                     ? QString("%1  (%2)").arg(bn).arg(dt.toString("yyyy-MM-dd HH:mm:ss"))
                     : QString::number(bn);
    }

    bool ok = false;
    QString newVer = QInputDialog::getText(
        this, "修改版本号",
        QString("当前版本号: %1\nBuild No:    %2\n\n请输入新的版本号 (如 1.0、1.2.3):")
            .arg(cur)
            .arg(bnText),
        QLineEdit::Normal, cur, &ok);
    if (!ok) return;
    newVer = newVer.trimmed();
    if (newVer.isEmpty()) {
        QMessageBox::warning(this, "提示", "版本号不能为空");
        return;
    }
    if (newVer == cur) return;
    project_.setVersion(newVer);
    markDirty();
    statusLabel_->setText(
        QString("版本号已修改为: %1（保存后写入 global.version，并刷新 build_no）")
            .arg(newVer));
}

void MainWindow::onResetView() {
    viewer_->resetCamera();
}

// ═══════════════════════════════════════════════════════════════════════
//  3D 刷新
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::refreshAllSubmaps() {
    for (const auto& id : project_.submapIds()) {
        refreshSubmap(id);
    }
}

void MainWindow::refreshSubmap(const QString& id) {
    auto* sm = project_.submap(id);
    if (!sm) return;

    viewer_->updateSubmap(
        id, sm->displayCloud(),
        project_.submapToRootMatrix(id),
        sm->displayOptions().color,
        sm->displayOptions().pointSize);
}

void MainWindow::refreshMarkers() {
    QVector<MarkerVertex> markers;
    QVector<MarkerInfo>   markerInfos;
    QVector<quint8>       markerEmphasis;
    QVector<MarkerVertex> arrowLines;

    constexpr float kArrowTotalLen   = 0.85f;
    constexpr float kArrowHeadLen    = 0.25f;
    constexpr float kArrowHeadWidth  = 0.14f;

    for (const auto& id : project_.submapIds()) {
        auto* sm = project_.submap(id);
        if (!sm) continue;
        Eigen::Matrix4d T = project_.submapToRootMatrix(id);
        Eigen::Vector3d smPlaneOrig = T.block<3,1>(0,3);
        Eigen::Vector3d smPlaneNorm = (T.block<3,3>(0,0) * Eigen::Vector3d::UnitZ()).normalized();
        QVector3D qPlaneOrig((float)smPlaneOrig.x(), (float)smPlaneOrig.y(), (float)smPlaneOrig.z());
        QVector3D qPlaneNorm((float)smPlaneNorm.x(), (float)smPlaneNorm.y(), (float)smPlaneNorm.z());

        // 语义点位 (默认黄色，当前选中项红色)
        if (sm->displayOptions().showSemanticPts) {
            for (auto it = sm->semanticPoints().begin(); it != sm->semanticPoints().end(); ++it) {
                const bool selected =
                    (id == selectedSemanticSmId_ && it.key() == selectedSemanticPtName_);
                const float r = 1.0f;
                const float g = selected ? 0.15f : 0.9f;
                const float b = selected ? 0.15f : 0.2f;
                for (const auto& pose : it.value().poses) {
                    Eigen::Vector4d pw = T * Eigen::Vector4d(
                        pose.position.x(), pose.position.y(), pose.position.z(), 1.0);
                    markers.append({(float)pw.x(), (float)pw.y(), (float)pw.z(), r, g, b});
                    markerInfos.append({it.key(), id, "semantic",
                                        QVector3D((float)pw.x(), (float)pw.y(), (float)pw.z()),
                                        qPlaneOrig, qPlaneNorm});
                    markerEmphasis.append(selected ? quint8(1) : quint8(0));

                    Eigen::Matrix4d Tworld = T * pose.toMatrix();
                    Eigen::Vector3d baseW(Tworld(0, 3), Tworld(1, 3), Tworld(2, 3));
                    Eigen::Matrix3d Rw = Tworld.block<3, 3>(0, 0);
                    Eigen::Vector3d forwardW = Rw * Eigen::Vector3d(1, 0, 0);
                    appendSemanticArrowLines(arrowLines, baseW, forwardW,
                                             kArrowTotalLen, kArrowHeadLen, kArrowHeadWidth,
                                             r, g, b);
                }
            }
        }

        // 传送门 (青色)
        if (sm->displayOptions().showPortals) {
            for (const auto& portal : sm->portals()) {
                // 传送门对应语义点位的位置
                auto* sp = sm->semanticPoint(portal.portalName);
                if (sp && !sp->poses.isEmpty()) {
                    const auto& pose = sp->poses.first();
                    Eigen::Vector4d pw = T * Eigen::Vector4d(
                        pose.position.x(), pose.position.y(), pose.position.z(), 1.0);
                    markers.append({(float)pw.x(), (float)pw.y(), (float)pw.z(),
                                    0.2f, 0.9f, 0.9f});
                    markerInfos.append({
                        QString("%1 → %2").arg(portal.portalName, portal.targetSubmap),
                        id, "portal",
                        QVector3D((float)pw.x(), (float)pw.y(), (float)pw.z()),
                        qPlaneOrig, qPlaneNorm});
                    markerEmphasis.append(quint8(0));
                }
            }
        }
    }

    viewer_->updateMarkers(markers, markerInfos, markerEmphasis);
    viewer_->updateSemanticArrows(arrowLines);
}

// ═══════════════════════════════════════════════════════════════════════
//  面板事件
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onSubmapSelected(const QString& id) {
    selectedSemanticSmId_.clear();
    selectedSemanticPtName_.clear();
    refreshMarkers();
    auto* sm = project_.submap(id);
    if (sm) propsPanel_->showSubmapProperties(sm);
}

void MainWindow::onSemanticPointSelected(const QString& smId, const QString& ptName) {
    selectedSemanticSmId_   = smId;
    selectedSemanticPtName_ = ptName;
    refreshMarkers();
    auto* sm = project_.submap(smId);
    if (sm) propsPanel_->showSemanticPointProperties(sm, ptName);
    if (submapPanel_->orbitCenterFollowsSemantic())
        setViewerOrbitCenterAtSemantic(smId, ptName);
}

void MainWindow::setViewerOrbitCenterAtSemantic(const QString& smId, const QString& ptName) {
    auto* sm = project_.submap(smId);
    if (!sm) return;
    auto* sp = sm->semanticPoint(ptName);
    if (!sp || sp->poses.isEmpty()) return;

    const Eigen::Matrix4d T = project_.submapToRootMatrix(smId);
    Eigen::Vector3d sum(0, 0, 0);
    for (const auto& pose : sp->poses) {
        Eigen::Vector4d pw = T * Eigen::Vector4d(
            pose.position.x(), pose.position.y(), pose.position.z(), 1.0);
        sum += Eigen::Vector3d(pw.x(), pw.y(), pw.z());
    }
    sum /= static_cast<double>(sp->poses.size());
    viewer_->lookAt(QVector3D(sum.x(), sum.y(), sum.z()));
}

void MainWindow::onSemanticPointChanged(const QString& smId, const QString& ptName) {
    if (selectedSemanticSmId_ == smId) selectedSemanticPtName_ = ptName;
    // Refresh markers and specific submap so the updated semantic point is visible immediately
    submapPanel_->rebuild(&project_);
    refreshMarkers();
    if (!smId.isEmpty()) refreshSubmap(smId);
    statusBar()->showMessage(QString("语义点已更新: %1").arg(ptName), 3000);
    markDirty();
}

void MainWindow::onPortalSelected(const QString& smId, int portalIdx) {
    selectedSemanticSmId_.clear();
    selectedSemanticPtName_.clear();
    refreshMarkers();
    auto* sm = project_.submap(smId);
    if (sm) propsPanel_->showPortalProperties(sm, portalIdx, project_.submapIds());
}

void MainWindow::onPortalPropertiesChanged(const QString& smId, int /*portalIdx*/) {
    submapPanel_->rebuild(&project_);
    refreshMarkers();
    if (!smId.isEmpty()) refreshSubmap(smId);
    markDirty();
    statusBar()->showMessage(QString("传送门已更新: %1").arg(smId), 3000);
}

void MainWindow::onSubmapVisibilityChanged(const QString& id, bool visible) {
    auto* sm = project_.submap(id);
    if (sm) {
        sm->displayOptions().visible = visible;
        viewer_->setSubmapVisible(id, visible);
        refreshMarkers();
    }
}

void MainWindow::onShowSemanticPointsChanged(const QString& id, bool show) {
    auto* sm = project_.submap(id);
    if (sm) {
        sm->displayOptions().showSemanticPts = show;
        refreshMarkers();
    }
}

void MainWindow::onShowPortalsChanged(const QString& id, bool show) {
    auto* sm = project_.submap(id);
    if (sm) {
        sm->displayOptions().showPortals = show;
        refreshMarkers();
    }
}

void MainWindow::onRouteVisibilityChanged(const QString& routeName, bool visible) {
    viewer_->setRouteVisible(routeName, visible);
}

void MainWindow::onRouteWaypointInserted(const QString& routeName, const QString& submapId,
                                          const QString& pointName, int insertIndex) {
    auto* r = project_.route(routeName);
    if (!r) return;

    RouteWaypoint wp;
    wp.submapId = submapId;
    wp.semanticPoint = pointName;

    if (insertIndex < 0 || insertIndex >= r->waypoints.size())
        r->waypoints.append(wp);
    else
        r->waypoints.insert(insertIndex, wp);

    markDirty();
    submapPanel_->rebuild(&project_);
    submapPanel_->expandRoute(routeName);
    refreshRouteLines();
    statusLabel_->setText(QString("已插入路线点: %1/%2 → %3 (位置 %4)")
                             .arg(submapId, pointName, routeName)
                             .arg(insertIndex + 1));
}

void MainWindow::onRouteRefInserted(const QString& routeName, const QString& nestedRouteName,
                                     int insertIndex) {
    auto* r = project_.route(routeName);
    if (!r) return;
    if (!project_.route(nestedRouteName)) return;

    RouteWaypoint wp;
    wp.routeRef = nestedRouteName;

    if (insertIndex < 0 || insertIndex >= r->waypoints.size())
        r->waypoints.append(wp);
    else
        r->waypoints.insert(insertIndex, wp);

    markDirty();
    submapPanel_->rebuild(&project_);
    submapPanel_->expandRoute(routeName);
    refreshRouteLines();
    statusLabel_->setText(QString("已插入嵌套路线: [%1] → %2 (位置 %3)")
                             .arg(nestedRouteName, routeName)
                             .arg(insertIndex + 1));
}

void MainWindow::onRouteWaypointRemoveRequested(const QString& routeName, int waypointIndex) {
    auto* r = project_.route(routeName);
    if (!r) return;
    if (waypointIndex < 0 || waypointIndex >= r->waypoints.size()) return;

    const auto w = r->waypoints[waypointIndex];
    r->waypoints.removeAt(waypointIndex);
    markDirty();
    submapPanel_->rebuild(&project_);
    submapPanel_->expandRoute(routeName);
    refreshRouteLines();
    statusLabel_->setText(QString("已从路线 %1 删除途经点 #%2 (%3 / %4)")
                              .arg(routeName)
                              .arg(waypointIndex + 1)
                              .arg(w.submapId)
                              .arg(w.semanticPoint));
}

// ═══════════════════════════════════════════════════════════════════════
//  路线折线刷新
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::refreshRouteLines() {
    static const QColor routeColors[] = {
        QColor(255, 100, 255),   // 粉色
        QColor(0, 255, 128),     // 荧光绿
        QColor(255, 200, 0),     // 金色
        QColor(0, 200, 255),     // 天蓝
        QColor(255, 128, 0),     // 橙色
        QColor(180, 100, 255),   // 紫色
    };
    int colorIdx = 0;

    viewer_->clearAllRouteLines();

    // 递归展开嵌套路线引用为扁平 waypoint 列表
    std::function<QVector<RouteWaypoint>(const Route&, QSet<QString>&)> flattenRoute =
        [&](const Route& route, QSet<QString>& visited) -> QVector<RouteWaypoint> {
        QVector<RouteWaypoint> flat;
        for (const auto& wp : route.waypoints) {
            if (wp.isRouteRef()) {
                if (visited.contains(wp.routeRef)) continue;  // 防止循环引用
                auto* nested = project_.route(wp.routeRef);
                if (!nested) continue;
                visited.insert(wp.routeRef);
                flat += flattenRoute(*nested, visited);
                visited.remove(wp.routeRef);
            } else {
                flat.append(wp);
            }
        }
        return flat;
    };

    for (auto it = project_.routes().begin(); it != project_.routes().end(); ++it) {
        QSet<QString> visited;
        visited.insert(it.key());
        auto flatWaypoints = flattenRoute(it.value(), visited);
        QVector<MarkerVertex> lineVerts;

        for (const auto& wp : flatWaypoints) {
            auto* sm = project_.submap(wp.submapId);
            if (!sm) continue;
            auto* sp = sm->semanticPoint(wp.semanticPoint);
            if (!sp || sp->poses.isEmpty()) continue;

            Eigen::Matrix4d T = project_.submapToRootMatrix(wp.submapId);
            const auto& pos = sp->poses.first().position;
            Eigen::Vector4d pw = T * Eigen::Vector4d(pos.x(), pos.y(), pos.z(), 1.0);

            const QColor& c = routeColors[colorIdx % 6];
            lineVerts.append({
                (float)pw.x(), (float)pw.y(), (float)pw.z(),
                (float)c.redF(), (float)c.greenF(), (float)c.blueF()
            });
        }

        if (lineVerts.size() >= 2) {
            viewer_->updateRouteLines(it.key(), lineVerts);
            viewer_->setRouteVisible(it.key(), false);
        }
        colorIdx++;
    }
}

void MainWindow::onSubmapPropertiesChanged(const QString& id) {
    refreshSubmap(id);
    refreshMarkers();
    markDirty();
    submapPanel_->rebuild(&project_);
}

void MainWindow::onColorChanged(const QString& id, const QColor& color) {
    Q_UNUSED(color);
    refreshSubmap(id);
}

void MainWindow::onGlobalVoxelSizeChanged(double size) {
    float vs = static_cast<float>(size);
    for (const auto& id : project_.submapIds()) {
        auto* sm = project_.submap(id);
        if (sm) {
            sm->displayOptions().voxelSize = vs;
            sm->updateDisplayCloud(vs);
        }
    }
    refreshAllSubmaps();
    statusLabel_->setText(QString("已将所有子地图降采样设为 %1 m").arg(vs, 0, 'f', 2));
}

// ═══════════════════════════════════════════════════════════════════════
//  新增 / 删除 Submap
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onAddSubmap() {
    AddSubmapDialog dlg(this);
    if (dlg.exec() == QDialog::Accepted && dlg.result()) {
        auto sm = dlg.result();
        if (project_.submap(sm->id())) {
            QMessageBox::warning(this, "错误",
                                 QString("ID '%1' 已存在").arg(sm->id()));
            return;
        }
        sm->displayOptions().color = nextColor();
        project_.addSubmap(sm);
        markDirty();
        submapPanel_->rebuild(&project_);
        refreshSubmap(sm->id());
        refreshMarkers();
        statusLabel_->setText("已添加子地图: " + sm->id());
    }
}

void MainWindow::onAddRoute() {
    bool ok;
    QString name = QInputDialog::getText(this, "新增路线", "路线名称:", QLineEdit::Normal, "", &ok);
    if (!ok || name.isEmpty()) return;
    // 检查是否已存在
    if (project_.route(name)) {
        QMessageBox::warning(this, "错误", QString("路线 '%1' 已存在").arg(name));
        return;
    }
    Route r;
    r.name = name;
    project_.addRoute(r);
    markDirty();
    submapPanel_->rebuild(&project_);
    refreshRouteLines();
    statusLabel_->setText("已添加路线: " + name);
}

void MainWindow::onRenameRoute(const QString& oldName, const QString& newName) {
    if (!project_.renameRoute(oldName, newName)) {
        QMessageBox::warning(this, "错误",
            QString("重命名失败：路线 '%1' 不存在或 '%2' 已被占用").arg(oldName, newName));
        return;
    }
    markDirty();
    submapPanel_->rebuild(&project_);
    submapPanel_->expandRoute(newName);
    refreshRouteLines();
    statusLabel_->setText(QString("已将路线 '%1' 重命名为 '%2'").arg(oldName, newName));
}

void MainWindow::onRenameSemanticPoint(const QString& smId,
                                       const QString& oldName,
                                       const QString& newName) {
    if (!project_.renameSemanticPoint(smId, oldName, newName)) {
        QMessageBox::warning(this, "错误",
            QString("重命名失败：点位 '%1' 不存在，或 '%2' 已存在")
                .arg(oldName, newName));
        return;
    }
    if (selectedSemanticSmId_ == smId && selectedSemanticPtName_ == oldName) {
        selectedSemanticPtName_ = newName;
    }
    markDirty();
    submapPanel_->rebuild(&project_);
    refreshSubmap(smId);
    refreshMarkers();
    refreshRouteLines();
    statusLabel_->setText(
        QString("已将语义点位 '%1' 重命名为 '%2'").arg(oldName, newName));
}

void MainWindow::onSetDefaultRoute(const QString& routeName) {
    if (routeName.isEmpty() || !project_.route(routeName)) return;
    project_.setDefaultRouteId(routeName);
    markDirty();
    submapPanel_->rebuild(&project_);
    statusLabel_->setText("已将 " + routeName + " 设置为 default_route");
}

void MainWindow::onSetDefaultSubmap(const QString& id) {
    if (id.isEmpty()) return;
    if (!project_.submap(id)) {
        QMessageBox::warning(this, "错误", "指定的子地图不存在");
        return;
    }
    if (!project_.isTreeRoot(id)) {
        QMessageBox::warning(this, "无法设为默认子地图",
            QString("子地图 「%1」 的父节点为 「%2」，不是 TF 树的根节点。\n"
                    "只有根节点（无父节点）才能设为 default_submap。\n"
                    "请先在右键菜单中清除其父节点关系。")
                .arg(id, project_.submap(id)->parentSubmapId()));
        return;
    }
    project_.setDefaultSubmapId(id);
    markDirty();
    submapPanel_->rebuild(&project_);
    statusLabel_->setText("已将 " + id + " 设置为 default_submap");
}

void MainWindow::onSetParentSubmap(const QString& submapId, const QString& parentId) {
    auto* sm = project_.submap(submapId);
    if (!sm) return;

    if (parentId.isEmpty()) {
        sm->setParentSubmapId(QString());
        markDirty();
        submapPanel_->rebuild(&project_);
        refreshSubmap(submapId);
        refreshMarkers();
        refreshRouteLines();
        statusLabel_->setText(QString("已清除 %1 的父节点").arg(submapId));
        return;
    }

    if (project_.wouldFormCycle(submapId, parentId)) {
        QMessageBox::warning(this, "无法设置父节点",
            QString("将 「%1」 的父节点设为 「%2」 会形成环路，操作已取消。")
                .arg(submapId, parentId));
        return;
    }

    sm->setParentSubmapId(parentId);
    markDirty();
    submapPanel_->rebuild(&project_);
    refreshAllSubmaps();
    refreshMarkers();
    refreshRouteLines();
    statusLabel_->setText(QString("已将 %1 的父节点设为 %2").arg(submapId, parentId));
}

void MainWindow::onShowTfTree() {
    if (project_.submapCount() == 0) {
        viewer_->setTfTreeVisible(false);
        QMessageBox::information(this, "TF 树", "当前没有子地图");
        return;
    }

    // 切换显示
    if (viewer_->isTfTreeVisible()) {
        viewer_->setTfTreeVisible(false);
        statusLabel_->setText("TF 树已隐藏");
        return;
    }

    // ── 收集根节点（parentSubmapId 为空或指向不存在的节点）──
    QStringList roots;
    for (const auto& id : project_.submapIds()) {
        auto* sm = project_.submap(id);
        if (!sm) continue;
        QString pid = sm->parentSubmapId();
        if (pid.isEmpty() || !project_.submap(pid))
            roots.append(id);
    }
    if (roots.isEmpty()) {
        roots = project_.submapIds();
    }

    // ── 计算每棵子树的叶宽度 ──
    std::function<int(const QString&)> subtreeWidth =
        [&](const QString& nodeId) -> int {
        QStringList children = project_.submapChildren(nodeId);
        if (children.isEmpty()) return 1;
        int w = 0;
        for (const auto& c : children)
            w += subtreeWidth(c);
        return w;
    };

    // ── 布局参数 ──
    const float nodeW   = 90.0f;   // 每个叶节点占用的水平宽度
    const float levelH  = 60.0f;   // 层间垂直间距
    const float padTop  = 40.0f;   // 顶部边距
    const float padLeft = 30.0f;   // 左侧边距

    // 总宽度
    int totalLeaves = 0;
    for (const auto& r : roots)
        totalLeaves += subtreeWidth(r);

    QVector<PointCloudViewer::TfTreeNode> nodes;
    QVector<PointCloudViewer::TfTreeEdge> edges;

    // ── 递归布局 ──
    std::function<void(const QString&, int, float, float)> layoutNode =
        [&](const QString& nodeId, int depth, float xStart, float xEnd) {
        float cx = (xStart + xEnd) * 0.5f;
        float cy = padTop + depth * levelH;

        bool isRoot = (project_.submap(nodeId) &&
                       project_.submap(nodeId)->parentSubmapId().isEmpty());
        nodes.append({nodeId, cx, cy, isRoot});

        QStringList children = project_.submapChildren(nodeId);
        if (children.isEmpty()) return;

        int totalW = 0;
        for (const auto& c : children)
            totalW += subtreeWidth(c);

        float childX = xStart;
        for (const auto& c : children) {
            int cw = subtreeWidth(c);
            float childXEnd = childX + (xEnd - xStart) * float(cw) / float(totalW);
            float childCx = (childX + childXEnd) * 0.5f;
            float childCy = padTop + (depth + 1) * levelH;

            edges.append({cx, cy, childCx, childCy});

            layoutNode(c, depth + 1, childX, childXEnd);
            childX = childXEnd;
        }
    };

    float xCursor = padLeft;
    for (const auto& rootId : roots) {
        int w = subtreeWidth(rootId);
        float xEnd = xCursor + w * nodeW;
        layoutNode(rootId, 0, xCursor, xEnd);
        xCursor = xEnd;
    }

    viewer_->updateTfTree(nodes, edges);
    viewer_->setTfTreeVisible(true);
    statusLabel_->setText(QString("TF 树已显示（%1 个节点）").arg(nodes.size()));
}

void MainWindow::onRemoveSubmap(const QString& id) {
    if (id == project_.defaultSubmapId()) {
        QMessageBox::warning(this, "提示", "不能删除 default_submap");
        return;
    }
    auto ret = QMessageBox::question(this, "确认", QString("确定删除子地图 '%1'?").arg(id));
    if (ret != QMessageBox::Yes) return;

    viewer_->removeSubmap(id);
    project_.removeSubmap(id);
    markDirty();
    submapPanel_->rebuild(&project_);
    refreshMarkers();
    propsPanel_->clear();
    statusLabel_->setText("已删除子地图: " + id);
}

// ═══════════════════════════════════════════════════════════════════════
//  添加语义点位 / 传送门（语义点用右侧属性栏；传送门用独立对话框新增）
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onAddSemanticPoint(const QString& smId) {
    auto* sm = project_.submap(smId);
    if (!sm) return;
    propsPanel_->beginNewSemanticPoint(sm);
    statusLabel_->setText(
        QString("在右侧「属性编辑」填写新语义点，可点「从点云选点」填入 position（子地图 %1，局部坐标）").arg(smId));
}

void MainWindow::onSemanticPickPositionRequested(const QString& smId) {
    auto* sm = project_.submap(smId);
    if (!sm) return;
    pendingAction_   = PendingSemanticPanelPick;
    pendingSubmapId_ = smId;

    Eigen::Matrix4d T = project_.submapToRootMatrix(smId);
    Eigen::Vector3d planeOrigin = T.block<3,1>(0,3);
    Eigen::Vector3d planeNormal = T.block<3,3>(0,0) * Eigen::Vector3d::UnitZ();
    viewer_->setPickPlane(planeOrigin, planeNormal);
    viewer_->setPickMode(PointCloudViewer::PickOnPlane);
    viewer_->setCursor(Qt::CrossCursor);
    statusLabel_->setText("在地图上单击以设置语义点 position（当前子地图平面）…");
}

void MainWindow::onAddPortal(const QString& smId) {
    auto* sm = project_.submap(smId);
    if (!sm) return;
    PortalEditorDialog dlg(sm, project_.submapIds(), this);
    if (dlg.exec() == QDialog::Accepted) {
        submapPanel_->rebuild(&project_);
        refreshMarkers();
        refreshSubmap(smId);
        markDirty();
        statusLabel_->setText(QString("已在子地图 %1 中添加传送门").arg(smId));
        statusBar()->showMessage("传送门已添加", 3000);
    }
}

void MainWindow::onAlignSubmap(const QString& smId) {
    auto* sm = project_.submap(smId);
    if (!sm) return;

    auto* dlg = new AlignSubmapDialog(&project_, smId, this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);

    auto pickConn = std::make_shared<QMetaObject::Connection>();
    auto srcPickConn = std::make_shared<QMetaObject::Connection>();
    auto origTransform = std::make_shared<RigidTransform>(sm->transform());
    auto previewApplied = std::make_shared<bool>(false);

    // ── 选择初始位置 (目标子地图) ──────────────────────────────────────
    connect(dlg, &AlignSubmapDialog::requestPickPoint, [this, smId, dlg, pickConn]() {
        pendingAction_   = PendingAlignGuess;
        pendingSubmapId_ = smId;
        viewer_->setPickMode(PointCloudViewer::PickGuessPosition);
        viewer_->setCursor(Qt::CrossCursor);
        statusLabel_->setText("请在目标子地图上点击选择初始位置...");

        if (*pickConn) QObject::disconnect(*pickConn);
        dlg->hide();

        *pickConn = connect(viewer_, &PointCloudViewer::pointPicked,
                        [this, dlg, pickConn](const Eigen::Vector3d& pos) {
                            dlg->setGuessPosition(pos);
                            QObject::disconnect(*pickConn);
                            viewer_->setPickMode(PointCloudViewer::NoPickMode);
                            viewer_->setCursor(Qt::ArrowCursor);
                            dlg->show();
                        });
    });

    // ── 选择源点云裁剪中心 ──────────────────────────────────────────
    connect(dlg, &AlignSubmapDialog::requestPickSourceCenter, [this, smId, dlg, srcPickConn]() {
        pendingAction_   = PendingAlignSourceCenter;
        pendingSubmapId_ = smId;
        viewer_->setPickMode(PointCloudViewer::PickGuessPosition);
        viewer_->setCursor(Qt::CrossCursor);
        statusLabel_->setText("请在源子地图上点击选择裁剪中心...");

        if (*srcPickConn) QObject::disconnect(*srcPickConn);
        dlg->hide();

        *srcPickConn = connect(viewer_, &PointCloudViewer::pointPicked,
                        [this, smId, dlg, srcPickConn](const Eigen::Vector3d& worldPos) {
                            Eigen::Vector3d localPos = worldPos;
                            {
                                Eigen::Matrix4d invT = project_.submapToRootMatrix(smId).inverse();
                                Eigen::Vector4d lp = invT * Eigen::Vector4d(worldPos.x(), worldPos.y(), worldPos.z(), 1.0);
                                localPos = lp.head<3>();
                            }
                            dlg->setSrcCropCenter(localPos);
                            QObject::disconnect(*srcPickConn);
                            viewer_->setPickMode(PointCloudViewer::NoPickMode);
                            viewer_->setCursor(Qt::ArrowCursor);
                            dlg->show();
                        });
    });

    // ── 裁剪框刷新 (可复用) ─────────────────────────────────────────────
    auto refreshCropBoxes = [this, smId, dlg]() {
        auto* s = project_.submap(smId);
        if (!s) return;

        Eigen::Vector3d guessPos = dlg->guessPosition();
        double rangeXY = dlg->searchRangeXY();
        double rangeZ  = dlg->searchRangeZ();
        Eigen::Vector3d tgtHalf(rangeXY, rangeXY, rangeZ);

        auto tgtVerts = buildWireframeBox(guessPos, tgtHalf, QColor(0, 200, 255));
        viewer_->updateCropBox("target_crop", tgtVerts);

        Eigen::Vector3d srcCenter = dlg->srcCropCenter();
        double halfCrop = dlg->cropSize() / 2.0;
        Eigen::Vector3d srcHalf(halfCrop, halfCrop, halfCrop);

        auto srcVerts = buildWireframeBox(srcCenter, srcHalf,
                                          QColor(255, 200, 0), project_.submapToRootMatrix(smId));
        viewer_->updateCropBox("source_crop", srcVerts);
    };

    auto cropBoxShowing = std::make_shared<bool>(false);

    // ── 预览裁剪框 ─────────────────────────────────────────────────────
    connect(dlg, &AlignSubmapDialog::requestPreview, [this, refreshCropBoxes, cropBoxShowing]() {
        refreshCropBoxes();
        *cropBoxShowing = true;
        statusLabel_->setText("已显示裁剪框预览 (青色=目标, 黄色=源)");
    });

    auto cleanup = [this, pickConn, srcPickConn]() {
        if (*pickConn) QObject::disconnect(*pickConn);
        if (*srcPickConn) QObject::disconnect(*srcPickConn);
        pendingAction_ = NoPending;
        viewer_->setPickMode(PointCloudViewer::NoPickMode);
        viewer_->setCursor(Qt::ArrowCursor);
        viewer_->clearCropBoxes();
    };

    connect(dlg, &QDialog::accepted, [this, smId, dlg, cleanup]() {
        const auto& result = dlg->matchResult();
        if (result.success) {
            if (auto* s = project_.submap(smId)) {
                s->setTransform(result.transform);
                refreshSubmap(smId);
                refreshMarkers();
                markDirty();
                submapPanel_->rebuild(&project_);
                statusLabel_->setText(QString("子地图 '%1' 对齐成功 (fitness=%2)")
                                         .arg(smId).arg(result.fitness, 0, 'f', 4));
            }
        }
        cleanup();
    });

    connect(dlg, &AlignSubmapDialog::previewAvailable,
            [this, smId, dlg, origTransform, previewApplied, cropBoxShowing, refreshCropBoxes]() {
        const auto& result = dlg->matchResult();
        if (result.success) {
            if (auto* s = project_.submap(smId)) {
                s->setTransform(result.transform);
                refreshSubmap(smId);
                refreshMarkers();
                *previewApplied = true;
                if (*cropBoxShowing) refreshCropBoxes();
                statusLabel_->setText("已预览对齐结果，点击 应用结果 保存，或 取消 回退");
            }
        }
    });

    connect(dlg, &QDialog::rejected, [this, smId, origTransform, previewApplied, cleanup]() {
        if (*previewApplied) {
            if (auto* s = project_.submap(smId)) {
                s->setTransform(*origTransform);
                refreshSubmap(smId);
                refreshMarkers();
                statusLabel_->setText("已取消预览，已回退原始变换");
            }
        }
        cleanup();
    });

    dlg->setModal(false);
    dlg->show();
}

void MainWindow::onBuildVprDatabase(const QString& smId) {
    if (!project_.submap(smId)) {
        QMessageBox::warning(this, "错误", "指定的 Submap 不存在: " + smId);
        return;
    }
    if (currentProjectDir_.isEmpty()) {
        QMessageBox::warning(this, "请先保存工程",
            "构建 VPR 数据库需要先确定输出目录。\n"
            "请先用「文件 → 另存为...」把工程保存到一个目录。");
        return;
    }

    // 数据库 (.npy/.npz/.txt/.log) 放在工程目录下，detect 直接吃这个目录
    QString outputDir = QDir(currentProjectDir_).absoluteFilePath("vpr/" + smId);

    // 图片缓存默认放在用户目录的 ~/.vpr/<projectName>/<submapId>/
    // —— 这样工程目录保持精简，缓存可独立清理
    QString projectName = QDir(currentProjectDir_).dirName();
    if (projectName.isEmpty()) projectName = "project";
    QString defaultCacheDir = QDir::home().absoluteFilePath(
        QString(".vpr/%1/%2").arg(projectName, smId));

    // 优先用本次会话内缓存；否则尝试从上次构建留下的 build_info.json 还原
    // 这样跨会话点 "更新" 时也能预填上次的参数
    BuildVprParams params;
    if (lastBuildVprParams_.contains(smId)) {
        params = lastBuildVprParams_.value(smId);
    } else {
        VprDatabaseInfo info = loadVprDatabaseInfo(outputDir);
        if (info.valid) params = toBuildVprParams(info);
    }

    BuildVprDatabaseDialog dlg(smId, outputDir, defaultCacheDir, params, this);
    dlg.exec();

    // 不论成功失败都记下用户填的参数，下次再开免去重填
    lastBuildVprParams_[smId] = dlg.currentParams();

    if (dlg.ranSuccessfully()) {
        statusLabel_->setText(QString("VPR 数据库已生成: %1").arg(outputDir));
        submapPanel_->rebuild(&project_);  // 刷新菜单项 (构建 → 更新)
    }
}

void MainWindow::onViewVprDatabase(const QString& smId) {
    if (!project_.submap(smId)) return;
    if (currentProjectDir_.isEmpty()) {
        QMessageBox::warning(this, "提示", "请先保存工程");
        return;
    }
    QString outputDir = QDir(currentProjectDir_).absoluteFilePath("vpr/" + smId);
    if (!QFileInfo(outputDir + "/database_meta.npz").exists()) {
        QMessageBox::information(this, "尚未构建",
            QString("Submap「%1」还未构建 VPR 数据库。").arg(smId));
        return;
    }
    VprDatabaseInfoDialog dlg(smId, outputDir, this);
    dlg.exec();
}

QVector<MarkerVertex> MainWindow::buildWireframeBox(
    const Eigen::Vector3d& center, const Eigen::Vector3d& half,
    const QColor& color, const Eigen::Matrix4d& transform) {

    float r = color.redF(), g = color.greenF(), b = color.blueF();

    Eigen::Vector3d corners[8] = {
        center + Eigen::Vector3d(-half.x(), -half.y(), -half.z()),
        center + Eigen::Vector3d( half.x(), -half.y(), -half.z()),
        center + Eigen::Vector3d( half.x(),  half.y(), -half.z()),
        center + Eigen::Vector3d(-half.x(),  half.y(), -half.z()),
        center + Eigen::Vector3d(-half.x(), -half.y(),  half.z()),
        center + Eigen::Vector3d( half.x(), -half.y(),  half.z()),
        center + Eigen::Vector3d( half.x(),  half.y(),  half.z()),
        center + Eigen::Vector3d(-half.x(),  half.y(),  half.z()),
    };

    for (int i = 0; i < 8; ++i) {
        Eigen::Vector4d p = transform * Eigen::Vector4d(corners[i].x(), corners[i].y(), corners[i].z(), 1.0);
        corners[i] = p.head<3>();
    }

    static const int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7},
    };

    QVector<MarkerVertex> verts;
    verts.reserve(24);
    for (const auto& e : edges) {
        const auto& a = corners[e[0]];
        const auto& c2 = corners[e[1]];
        verts.append({(float)a.x(), (float)a.y(), (float)a.z(), r, g, b});
        verts.append({(float)c2.x(), (float)c2.y(), (float)c2.z(), r, g, b});
    }
    return verts;
}

// ═══════════════════════════════════════════════════════════════════════
//  拾取回调
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onPointPicked(const Eigen::Vector3d& pos) {
    auto* sm = project_.submap(pendingSubmapId_);

    switch (pendingAction_) {
    case PendingSemanticPanelPick: {
        if (!sm) break;
        Eigen::Matrix4d invT = project_.submapToRootMatrix(pendingSubmapId_).inverse();
        Eigen::Vector4d local = invT * Eigen::Vector4d(pos.x(), pos.y(), pos.z(), 1.0);
        propsPanel_->setSemanticPositionLocal(local.x(), local.y(), 0.0);
        statusLabel_->setText(QString("已填入 position（局部坐标）: (%1, %2, 0.00)，可在右侧面板继续编辑后点「应用修改」")
                                  .arg(local.x(), 0, 'f', 2)
                                  .arg(local.y(), 0, 'f', 2));
        break;
    }
    case PendingAlignGuess:
        statusLabel_->setText(QString("初始位置: (%1, %2, %3)")
                                 .arg(pos.x(), 0, 'f', 2)
                                 .arg(pos.y(), 0, 'f', 2)
                                 .arg(pos.z(), 0, 'f', 2));
        break;
    case PendingAlignSourceCenter:
        statusLabel_->setText(QString("源裁剪中心: (%1, %2, %3)")
                                 .arg(pos.x(), 0, 'f', 2)
                                 .arg(pos.y(), 0, 'f', 2)
                                 .arg(pos.z(), 0, 'f', 2));
        break;
    default:
        break;
    }

    pendingAction_ = NoPending;
}

void MainWindow::onMarkerDragFinished(const QString& submapId, const QString& name, const Eigen::Vector3d& newWorldPos) {
    auto* sm = project_.submap(submapId);
    if (!sm) return;
    auto* sp = sm->semanticPoint(name);
    if (!sp) return;

    Eigen::Matrix4d invT = project_.submapToRootMatrix(submapId).inverse();
    Eigen::Vector4d lp = invT * Eigen::Vector4d(newWorldPos.x(), newWorldPos.y(), newWorldPos.z(), 1.0);
    Eigen::Vector3d localPos(lp.x(), lp.y(), 0.0);

    if (!sp->poses.isEmpty()) {
        markerDragUndo_ = {true, submapId, name, sp->poses[0].position};
        sp->poses[0].position = localPos;
    } else {
        markerDragUndo_.valid = false;
        Pose p; p.position = localPos; sp->poses.append(p);
    }
    markDirty();
    refreshMarkers();
    refreshSubmap(submapId);
    if (submapId == selectedSemanticSmId_ && name == selectedSemanticPtName_)
        propsPanel_->showSemanticPointProperties(sm, name);
    statusBar()->showMessage(QString("已移动语义点 %1 到新位置 (Ctrl+Z 撤销)").arg(name), 5000);
}

void MainWindow::undoMarkerDrag() {
    if (!markerDragUndo_.valid) {
        statusBar()->showMessage("没有可撤销的拖拽操作", 3000);
        return;
    }
    auto* sm = project_.submap(markerDragUndo_.submapId);
    if (!sm) return;
    auto* sp = sm->semanticPoint(markerDragUndo_.pointName);
    if (!sp || sp->poses.isEmpty()) return;

    sp->poses[0].position = markerDragUndo_.oldPosition;
    QString undoneSmId = markerDragUndo_.submapId;
    QString undonePtName = markerDragUndo_.pointName;
    markerDragUndo_.valid = false;

    markDirty();
    refreshMarkers();
    refreshSubmap(undoneSmId);
    if (undoneSmId == selectedSemanticSmId_ && undonePtName == selectedSemanticPtName_)
        propsPanel_->showSemanticPointProperties(sm, undonePtName);
    statusBar()->showMessage(
        QString("已撤销语义点 %1 的拖拽").arg(undonePtName), 3000);
}

} // namespace me
