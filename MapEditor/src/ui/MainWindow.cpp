#include "ui/MainWindow.h"
#include "ui/AddSubmapDialog.h"
#include "ui/AlignSubmapDialog.h"
#include "ui/PortalEditorDialog.h"

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

namespace me {

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
    setWindowTitle("MapEditor — 地图编辑器");
    resize(1400, 900);
    setupUI();
    setupMenuBar();
    connectSignals();
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
    fileMenu->addAction("退出", qApp, &QApplication::quit, QKeySequence::Quit);

    auto* viewMenu = menuBar()->addMenu("视图(&V)");
    viewMenu->addAction("重置视角", this, &MainWindow::onResetView, QKeySequence("R"));

    auto* editMenu = menuBar()->addMenu("编辑(&E)");
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
    connect(submapPanel_, &SubmapPanel::routeWaypointRemoveRequested,
            this, &MainWindow::onRouteWaypointRemoveRequested);

    connect(submapPanel_, &SubmapPanel::addSubmapRequested,
            this, &MainWindow::onAddSubmap);
        connect(submapPanel_, &SubmapPanel::addRouteRequested,
            this, &MainWindow::onAddRoute);
    connect(submapPanel_, &SubmapPanel::removeSubmapRequested,
            this, &MainWindow::onRemoveSubmap);
    connect(submapPanel_, &SubmapPanel::addSemanticPointRequested,
            this, &MainWindow::onAddSemanticPoint);
    connect(submapPanel_, &SubmapPanel::addPortalRequested,
            this, &MainWindow::onAddPortal);
    connect(submapPanel_, &SubmapPanel::alignSubmapRequested,
            this, &MainWindow::onAlignSubmap);
        connect(submapPanel_, &SubmapPanel::setDefaultSubmapRequested,
            this, &MainWindow::onSetDefaultSubmap);

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
    setWindowTitle(QString("MapEditor — %1").arg(QDir(path).dirName()));
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
    onSave();
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
        sm->transformMatrix(),
        sm->displayOptions().color,
        sm->displayOptions().pointSize);
}

void MainWindow::refreshMarkers() {
    QVector<MarkerVertex> markers;
    QVector<MarkerInfo>   markerInfos;
    QVector<quint8>       markerEmphasis;

    for (const auto& id : project_.submapIds()) {
        auto* sm = project_.submap(id);
        if (!sm) continue;
        Eigen::Matrix4d T = sm->transformMatrix();

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
                                        QVector3D((float)pw.x(), (float)pw.y(), (float)pw.z())});
                    markerEmphasis.append(selected ? quint8(1) : quint8(0));
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
                        QVector3D((float)pw.x(), (float)pw.y(), (float)pw.z())});
                    markerEmphasis.append(quint8(0));
                }
            }
        }
    }

    viewer_->updateMarkers(markers, markerInfos, markerEmphasis);
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
}

void MainWindow::onSemanticPointChanged(const QString& smId, const QString& ptName) {
    if (selectedSemanticSmId_ == smId) selectedSemanticPtName_ = ptName;
    // Refresh markers and specific submap so the updated semantic point is visible immediately
    submapPanel_->rebuild(&project_);
    refreshMarkers();
    if (!smId.isEmpty()) refreshSubmap(smId);
    statusBar()->showMessage(QString("语义点已更新: %1").arg(ptName), 3000);
    project_.setDirty();
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
    project_.setDirty();
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

    project_.setDirty();
    submapPanel_->rebuild(&project_);
    submapPanel_->expandRoute(routeName);
    refreshRouteLines();
    statusLabel_->setText(QString("已插入路线点: %1/%2 → %3 (位置 %4)")
                             .arg(submapId, pointName, routeName)
                             .arg(insertIndex + 1));
}

void MainWindow::onRouteWaypointRemoveRequested(const QString& routeName, int waypointIndex) {
    auto* r = project_.route(routeName);
    if (!r) return;
    if (waypointIndex < 0 || waypointIndex >= r->waypoints.size()) return;

    const auto& w = r->waypoints[waypointIndex];
    r->waypoints.removeAt(waypointIndex);
    project_.setDirty();
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

    for (auto it = project_.routes().begin(); it != project_.routes().end(); ++it) {
        const auto& route = it.value();
        QVector<MarkerVertex> lineVerts;

        for (const auto& wp : route.waypoints) {
            auto* sm = project_.submap(wp.submapId);
            if (!sm) continue;
            auto* sp = sm->semanticPoint(wp.semanticPoint);
            if (!sp || sp->poses.isEmpty()) continue;

            Eigen::Matrix4d T = sm->transformMatrix();
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
        }
        colorIdx++;
    }
}

void MainWindow::onSubmapPropertiesChanged(const QString& id) {
    refreshSubmap(id);
    refreshMarkers();
    project_.setDirty();
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
    project_.setDirty();
    submapPanel_->rebuild(&project_);
    refreshRouteLines();
    statusLabel_->setText("已添加路线: " + name);
}

void MainWindow::onSetDefaultSubmap(const QString& id) {
    if (id.isEmpty()) return;
    if (!project_.submap(id)) {
        QMessageBox::warning(this, "错误", "指定的子地图不存在");
        return;
    }
    project_.setDefaultSubmapId(id);
    project_.setDirty();
    submapPanel_->rebuild(&project_);
    statusLabel_->setText("已将 " + id + " 设置为 default_submap");
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
    if (!project_.submap(smId)) return;
    pendingAction_   = PendingSemanticPanelPick;
    pendingSubmapId_ = smId;
    viewer_->setPickMode(PointCloudViewer::PickPointOnCloud);
    viewer_->setCursor(Qt::CrossCursor);
    statusLabel_->setText("在点云上单击以设置语义点 position（子地图局部坐标）…");
}

void MainWindow::onAddPortal(const QString& smId) {
    auto* sm = project_.submap(smId);
    if (!sm) return;
    PortalEditorDialog dlg(sm, project_.submapIds(), this);
    if (dlg.exec() == QDialog::Accepted) {
        submapPanel_->rebuild(&project_);
        refreshMarkers();
        refreshSubmap(smId);
        project_.setDirty();
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
                            auto* s = project_.submap(smId);
                            Eigen::Vector3d localPos = worldPos;
                            if (s) {
                                Eigen::Matrix4d invT = s->transformMatrix().inverse();
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
                                          QColor(255, 200, 0), s->transformMatrix());
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
                project_.setDirty();
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
        Eigen::Matrix4d invT = sm->transformMatrix().inverse();
        Eigen::Vector4d local = invT * Eigen::Vector4d(pos.x(), pos.y(), pos.z(), 1.0);
        propsPanel_->setSemanticPositionLocal(local.x(), local.y(), local.z());
        statusLabel_->setText(QString("已填入 position（局部）: (%1, %2, %3)，可在右侧面板继续编辑后点「应用修改」")
                                  .arg(local.x(), 0, 'f', 2)
                                  .arg(local.y(), 0, 'f', 2)
                                  .arg(local.z(), 0, 'f', 2));
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

void MainWindow::onMarkerDragFinished(const QString& submapId, const QString& name, const Eigen::Vector3d& newPos) {
    // Update semantic point pose in project and refresh display
    auto* sm = project_.submap(submapId);
    if (!sm) return;
    auto* sp = sm->semanticPoint(name);
    if (!sp) return;
    // For simplicity assume single_pose type and update first pose
    if (!sp->poses.isEmpty()) {
        sp->poses[0].position = newPos;
    } else {
        Pose p; p.position = newPos; sp->poses.append(p);
    }
    project_.setDirty();
    refreshMarkers();
    refreshSubmap(submapId);
    statusBar()->showMessage(QString("已移动语义点 %1 到新位置").arg(name), 3000);
}

} // namespace me
