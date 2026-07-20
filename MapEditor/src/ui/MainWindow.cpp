#include "ui/MainWindow.h"
#include "ui/AddSubmapDialog.h"
#include "ui/AlignSubmapDialog.h"
#include "ui/BuildVprDatabaseDialog.h"
#include "ui/PortalEditorDialog.h"
#include "ui/VprDatabaseInfoDialog.h"
#include "ui/VprImageViewer.h"
#include "ui/SceneAlignDialog.h"

#include "ui/SceneAlignView.h"

#include <QMenuBar>
#include <QToolBar>
#include <QDockWidget>
#include <QTabWidget>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QFileInfo>
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
#include <pcl/common/transforms.h>
#include <QSet>
#include <QColor>
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
    // ── 中心: 堆叠视图 [0]=场景总览(正交叠加)  [1]=3D 活跃地图 ────────
    sceneView_ = new SceneAlignView();
    viewer_    = new PointCloudViewer();
    centralStack_ = new QStackedWidget();
    centralStack_->addWidget(sceneView_);   // index 0
    centralStack_->addWidget(viewer_);      // index 1
    centralStack_->setCurrentIndex(1);
    setCentralWidget(centralStack_);

    // ── 左侧: 场景地图 / SLAM地图 两个 Tab ───────────────────────────
    //   · 场景地图：OSM 底图 + 各 SLAM 地图 + 全局路线（跨地图）
    //   · SLAM地图：活跃地图内部 submap + 其内部路线
    scenePanel_  = new ScenePanel();
    submapPanel_ = new SubmapPanel();
    leftTabs_ = new QTabWidget();
    leftTabs_->addTab(scenePanel_,  "场景地图");   // kSceneTab = 0
    leftTabs_->addTab(submapPanel_, "SLAM地图");   // kSlamTab  = 1
    leftTabs_->setCurrentIndex(kSlamTab);
    connect(leftTabs_, &QTabWidget::currentChanged, this, &MainWindow::onLeftTabChanged);
    auto* leftDock = new QDockWidget("场景管理", this);
    leftDock->setWidget(leftTabs_);
    leftDock->setMinimumWidth(280);
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
    fileMenu->addAction("打开场景目录...", this, &MainWindow::onOpenScene, QKeySequence::Open);
    fileMenu->addAction("保存场景 (目录)", this, &MainWindow::onSave, QKeySequence::Save);
    fileMenu->addAction("场景另存为 (目录)...", this, &MainWindow::onSaveAs, QKeySequence("Ctrl+Shift+S"));
    fileMenu->addSeparator();
    fileMenu->addAction("导入单张地图目录 (multifloor.json)...", this, &MainWindow::onOpen);
    fileMenu->addAction("导入 OSM/PBF 底图...", this, &MainWindow::onImportOsm);
    fileMenu->addAction("导入 SLAM 地图...", this, &MainWindow::onImportSlamMap);
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

    auto* sceneMenu = menuBar()->addMenu("场景(&S)");
    sceneMenu->addAction("场景对齐 (OSM + SLAM)...", this,
                         &MainWindow::onOpenSceneAlign, QKeySequence("Ctrl+M"));

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

    // VPR 视觉特征相关
    connect(submapPanel_, &SubmapPanel::showVprFeaturesChanged,
            this, &MainWindow::onShowVprFeaturesChanged);
    connect(submapPanel_, &SubmapPanel::deleteSelectedVprFeaturesRequested,
            this, &MainWindow::onDeleteSelectedVprFeaturesRequested);
    connect(viewer_, &PointCloudViewer::vprPointClicked,
            this, &MainWindow::onVprPointClicked);
    connect(viewer_, &PointCloudViewer::vprSelectionChanged,
            this, &MainWindow::onVprSelectionChanged);
        connect(submapPanel_, &SubmapPanel::setDefaultSubmapRequested,
            this, &MainWindow::onSetDefaultSubmap);
    connect(submapPanel_, &SubmapPanel::setParentSubmapRequested,
            this, &MainWindow::onSetParentSubmap);
    connect(submapPanel_, &SubmapPanel::showTfTreeRequested,
            this, &MainWindow::onShowTfTree);

    // ── 场景地图 Tab ──────────────────────────────────────────────────
    connect(scenePanel_, &ScenePanel::importOsmRequested,
            this, &MainWindow::onImportOsm);
    connect(scenePanel_, &ScenePanel::addSlamMapRequested,
            this, &MainWindow::onImportSlamMap);
    connect(scenePanel_, &ScenePanel::removeSlamMapRequested,
            this, &MainWindow::onRemoveSlamMap);
    connect(scenePanel_, &ScenePanel::renameSlamMapRequested,
            this, &MainWindow::onRenameSlamMap);
    connect(scenePanel_, &ScenePanel::exportSlamMapRequested,
            this, &MainWindow::onExportSlamMap);
    connect(scenePanel_, &ScenePanel::slamMapActivated,
            this, &MainWindow::onSlamMapActivated);
    connect(scenePanel_, &ScenePanel::slamMapSelected,
            this, &MainWindow::showSlamGeoInPanel);
    connect(scenePanel_, &ScenePanel::osmSelected,
            this, &MainWindow::onOsmSelected);
    connect(scenePanel_, &ScenePanel::orthoVisibilityChanged,
            this, &MainWindow::onOrthoVisibilityChanged);

    // 在总览里点击足迹选中 → 右侧属性栏显示该地图 geo_anchor
    connect(sceneView_, &SceneAlignView::selectionChanged, this, [this](int index) {
        if (index < 0 || index >= sceneViewRowToId_.size()) return;
        showSlamGeoInPanel(sceneViewRowToId_[index]);
    });
    // 鼠标在总览上移动 → 右侧属性栏更新经纬度
    connect(sceneView_, &SceneAlignView::mouseGeoMoved, this,
            [this](double lon, double lat, bool valid) {
        propsPanel_->setMouseGeo(lon, lat, valid);
    });

    // 在总览里拖拽足迹 → 写回该地图 geo_anchor（并同步属性栏编辑框）
    connect(sceneView_, &SceneAlignView::anchorEdited, this,
            [this](int index, const GeoAnchor& anchor) {
        if (index < 0 || index >= sceneViewRowToId_.size()) return;
        const QString id = sceneViewRowToId_[index];
        auto* e = scene_.mapById(id);
        if (!e) return;
        e->anchor = anchor;
        propsPanel_->updateSlamGeoAnchor(id, anchor);
        scenePanel_->rebuild(&scene_);
        markDirty();
    });

    // 右侧属性栏编辑 geo_anchor → 写回场景 + 刷新总览足迹
    connect(propsPanel_, &PropertiesPanel::slamGeoAnchorChanged,
            this, &MainWindow::onSlamGeoAnchorEdited);

    // 兼容：SLAM地图 Tab 内若仍触发这些信号
    connect(submapPanel_, &SubmapPanel::slamMapSelected,
            this, &MainWindow::onSlamMapSelected);
    connect(submapPanel_, &SubmapPanel::importOsmRequested,
            this, &MainWindow::onImportOsm);
    connect(submapPanel_, &SubmapPanel::addSlamMapRequested,
            this, &MainWindow::onImportSlamMap);
    connect(submapPanel_, &SubmapPanel::removeSlamMapRequested,
            this, &MainWindow::onRemoveSlamMap);
    connect(submapPanel_, &SubmapPanel::alignSlamMapRequested,
            this, &MainWindow::onAlignSlamMap);

    // ── 全局路线（在「场景地图」Tab 下管理）──────────────────────────
    connect(scenePanel_, &ScenePanel::addGlobalRouteRequested,
            this, &MainWindow::onAddGlobalRoute);
    connect(scenePanel_, &ScenePanel::removeGlobalRouteRequested,
            this, &MainWindow::onRemoveGlobalRoute);
    connect(scenePanel_, &ScenePanel::renameGlobalRouteRequested,
            this, &MainWindow::onRenameGlobalRoute);
    connect(scenePanel_, &ScenePanel::addGlobalWaypointRequested,
            this, &MainWindow::onAddGlobalWaypoint);
    connect(scenePanel_, &ScenePanel::removeGlobalWaypointRequested,
            this, &MainWindow::onRemoveGlobalWaypoint);

    // ── 场景语义点位（OSM 底图上，经纬度 + 航向角）───────────────────
    connect(scenePanel_, &ScenePanel::addScenePointRequested,
            this, &MainWindow::onAddScenePoint);
    connect(scenePanel_, &ScenePanel::scenePointSelected,
            this, &MainWindow::onScenePointSelected);
    connect(scenePanel_, &ScenePanel::removeScenePointRequested,
            this, &MainWindow::onRemoveScenePoint);
    connect(scenePanel_, &ScenePanel::renameScenePointRequested,
            this, &MainWindow::onRenameScenePoint);
    connect(sceneView_, &SceneAlignView::scenePointPlaced,
            this, &MainWindow::onScenePointPlacedOnMap);
    connect(sceneView_, &SceneAlignView::scenePointSelected,
            this, &MainWindow::onScenePointSelected);
    connect(sceneView_, &SceneAlignView::scenePointMoved,
            this, &MainWindow::onScenePointMovedOnMap);
    connect(propsPanel_, &PropertiesPanel::sceneSemanticPointChanged,
            this, &MainWindow::onSceneSemanticPointApply);
    connect(propsPanel_, &PropertiesPanel::pickScenePositionRequested, this, [this]() {
        scenePointPickForName_ = selectedScenePoint_;   // 回填当前选中点位
        showLeftTab(kSceneTab);
        sceneView_->setAddPointMode(true);
        statusLabel_->setText("请在底图上点选新的位置");
    });

    // ── 内部路线：删除（走活跃地图）──────────────────────────────────
    connect(submapPanel_, &SubmapPanel::removeRouteRequested, this,
            [this](const QString& name) { onRemoveInternalRoute(activeMapId(), name); });

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

    // 切工程：清空 VPR 缓存 & 视图里的紫色点 & 选中集合
    {
        QStringList loadedSubmaps = vprDbs_.keys();
        vprDbs_.clear();
        vprSelection_.clear();
        for (const auto& sm : loadedSubmaps) viewer_->removeVprPoints(sm);
    }

    // 为每个 submap 分配颜色
    for (const auto& id : project_.submapIds()) {
        auto* sm = project_.submap(id);
        if (sm) {
            sm->displayOptions().color = nextColor();
        }
    }

    // 把打开的单张地图包装为场景中的一张 SLAM 地图（OSM 未导入）
    scene_.clear();
    {
        SlamMapEntry e;
        e.id   = scene_.uniqueMapId(QFileInfo(path).completeBaseName());
        e.name = QFileInfo(path).completeBaseName();
        e.pcdBaseDir = path;
        e.anchor.valid = false;
        e.project = std::make_shared<MapProject>(project_);
        scene_.addMap(e);
        scene_.setActiveIndex(0);
    }

    refreshScenePanels();
    refreshAllSubmaps();
    refreshMarkers();
    refreshRouteLines();
    viewer_->resetCamera();

    // 自动显示已构建 VPR 数据库且勾选可见的 submap 视觉特征点
    for (const auto& smId : project_.submapIds()) {
        auto* sm = project_.submap(smId);
        if (!sm || !sm->displayOptions().showVprFeatures) continue;
        QString dbDir = QDir(currentProjectDir_).absoluteFilePath("vpr/" + smId);
        if (!VprDatabaseData::dirHasDatabase(dbDir)) continue;
        onShowVprFeaturesChanged(smId, true);
    }

    statusLabel_->setText(QString("已加载: %1 | %2 个子地图 | %3 条路线")
                              .arg(path)
                              .arg(project_.submapCount())
                              .arg(project_.routes().size()));
    project_.setDirty(false);
    updateWindowTitle();
}

// 统一保存到「场景目录」：目录内含 scene.json + 复制进来的 OSM/PBF 底图 + 各 SLAM
// 地图的 PCD；scene.json 内资源引用一律用相对路径，便于整目录迁移。
bool MainWindow::saveSceneToDir(const QString& dir) {
    syncActiveEntryFromProject();
    statusLabel_->setText("保存场景中（复制底图/点云）...");
    QApplication::processEvents();
    if (!scene_.saveToDir(dir)) {
        QMessageBox::critical(this, "错误", "保存失败: " + dir);
        return false;
    }
    project_.setDirty(false);
    currentProjectDir_ = dir;
    updateWindowTitle();
    statusLabel_->setText(QString("已保存场景目录: %1  (%2 张地图, %3 条全局路线)")
                              .arg(dir)
                              .arg(scene_.mapCount())
                              .arg(scene_.globalRoutes().size()));
    return true;
}

void MainWindow::onSave() {
    // 已有场景目录 → 直接覆盖保存；否则走「另存为」让用户选目录
    const QString dir = scene_.sceneDir();
    if (dir.isEmpty()) {
        onSaveAs();
        return;
    }
    saveSceneToDir(dir);
}

void MainWindow::onSaveAs() {
    QString start = scene_.sceneDir();
    if (start.isEmpty()) start = currentProjectDir_;
    QString dir = QFileDialog::getExistingDirectory(
        this, "选择场景保存目录（将写入 scene.json 并复制底图/点云）", start,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (dir.isEmpty()) return;
    saveSceneToDir(dir);
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

void MainWindow::onOpenSceneAlign() {
    showLeftTab(kSceneTab);
}

// ═══════════════════════════════════════════════════════════════════════
//  场景：活跃地图与 project_ 的同步
// ═══════════════════════════════════════════════════════════════════════

QString MainWindow::activeMapId() const {
    if (scene_.activeIndex() >= 0 && scene_.activeIndex() < scene_.maps().size())
        return scene_.maps()[scene_.activeIndex()].id;
    return QString();
}

void MainWindow::syncActiveEntryFromProject() {
    auto* e = scene_.activeMap();
    if (!e) return;
    if (!e->project) e->project = std::make_shared<MapProject>();
    *e->project = project_;
    if (!currentProjectDir_.isEmpty()) e->pcdBaseDir = currentProjectDir_;
}

void MainWindow::loadProjectFromActiveEntry() {
    auto* e = scene_.activeMap();
    if (!e || !e->project) return;
    project_ = *e->project;
    currentProjectDir_ = e->pcdBaseDir;
}

MapProject* MainWindow::projectForMap(const QString& mapId) {
    if (mapId == activeMapId()) return &project_;
    auto* e = scene_.mapById(mapId);
    return e ? e->project.get() : nullptr;
}

void MainWindow::clearViewerActiveMap() {
    for (const auto& id : project_.submapIds()) {
        viewer_->removeSubmap(id);
        viewer_->removeVprPoints(id);
    }
    viewer_->clearAllRouteLines();
    vprDbs_.clear();
    vprSelection_.clear();
}

void MainWindow::refreshScenePanels() {
    syncActiveEntryFromProject();
    scenePanel_->rebuild(&scene_);
    submapPanel_->rebuildActiveMap(scene_.activeMap() ? &project_ : nullptr);
    sceneView_->setScenePoints(scene_.osmPoints());
    sceneView_->setSelectedScenePoint(selectedScenePoint_);
    sceneOverviewDirty_ = true;
    if (leftTabs_ && leftTabs_->currentIndex() == kSceneTab) rebuildSceneOverview();
}

void MainWindow::refreshActiveMapView() {
    colorIndex_ = 0;
    for (const auto& id : project_.submapIds()) {
        auto* sm = project_.submap(id);
        if (sm && !sm->displayOptions().color.isValid())
            sm->displayOptions().color = nextColor();
    }
    refreshAllSubmaps();
    refreshMarkers();
    refreshRouteLines();
}

// ═══════════════════════════════════════════════════════════════════════
//  场景：导入 OSM / SLAM 地图
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onImportOsm() {
    QString path = QFileDialog::getOpenFileName(
        this, "导入 OSM/PBF 底图", currentProjectDir_,
        "OSM 地图 (*.osm *.pbf *.osm.pbf);;OSM XML (*.osm);;OSM PBF (*.pbf *.osm.pbf);;所有文件 (*)");
    if (path.isEmpty()) return;

    statusLabel_->setText("加载底图中...");
    QApplication::processEvents();

    auto osm = std::make_shared<OsmData>();
    if (!osm->load(path) || osm->isEmpty()) {
        QMessageBox::critical(this, "错误", "无法解析 OSM 底图: " + path);
        statusLabel_->setText("底图加载失败");
        return;
    }
    scene_.setOsm(osm, path);
    refreshScenePanels();
    // 切到「场景地图」Tab，主视图即显示 OSM 底图 + 各 SLAM 地图正交投影
    showLeftTab(kSceneTab);
    statusLabel_->setText(QString("已导入底图: %1  (中心 lon %2, lat %3)")
                              .arg(QFileInfo(path).fileName())
                              .arg(scene_.originLon(), 0, 'f', 6)
                              .arg(scene_.originLat(), 0, 'f', 6));
}

void MainWindow::onImportSlamMap() {
    QString path = QFileDialog::getExistingDirectory(
        this, "选择 SLAM 地图工程目录", currentProjectDir_,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (path.isEmpty()) return;

    statusLabel_->setText("加载 SLAM 地图中...");
    QApplication::processEvents();

    auto proj = std::make_shared<MapProject>();
    if (!proj->load(path)) {
        QMessageBox::critical(this, "错误", "无法加载 SLAM 地图: " + path);
        statusLabel_->setText("SLAM 地图加载失败");
        return;
    }

    // 先把当前活跃地图的编辑写回，再追加新地图并切换为活跃
    syncActiveEntryFromProject();
    clearViewerActiveMap();

    SlamMapEntry e;
    e.id   = scene_.uniqueMapId(QFileInfo(path).completeBaseName());
    e.name = QFileInfo(path).completeBaseName();
    e.pcdBaseDir = path;
    e.project = proj;
    e.anchor.valid = true;
    if (scene_.hasOrigin()) { e.anchor.lon = scene_.originLon(); e.anchor.lat = scene_.originLat(); }
    scene_.addMap(e);   // 设为活跃

    loadProjectFromActiveEntry();
    for (const auto& id : project_.submapIds()) {
        auto* sm = project_.submap(id);
        if (sm) sm->displayOptions().color = nextColor();
    }
    refreshScenePanels();
    refreshActiveMapView();
    viewer_->resetCamera();
    showLeftTab(kSlamTab);
    markDirty();
    statusLabel_->setText(QString("已导入 SLAM 地图: %1  (%2 个子地图)")
                              .arg(e.name).arg(proj->submapCount()));
}

// ═══════════════════════════════════════════════════════════════════════
//  场景：选择 / 移除 / 对齐 SLAM 地图
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onSlamMapSelected(const QString& slamMapId) {
    int idx = scene_.indexOfMap(slamMapId);
    if (idx < 0 || idx == scene_.activeIndex()) return;

    syncActiveEntryFromProject();
    clearViewerActiveMap();
    selectedSemanticSmId_.clear();
    selectedSemanticPtName_.clear();

    scene_.setActiveIndex(idx);
    loadProjectFromActiveEntry();

    refreshScenePanels();
    refreshActiveMapView();
    viewer_->resetCamera();
    updateWindowTitle();
    statusLabel_->setText(QString("当前编辑地图: %1").arg(scene_.maps()[idx].name));
}

void MainWindow::onRemoveSlamMap(const QString& slamMapId) {
    int idx = scene_.indexOfMap(slamMapId);
    if (idx < 0) return;
    if (QMessageBox::question(this, "移除地图",
            QString("确定从场景移除 SLAM 地图「%1」？（不会删除磁盘文件）")
                .arg(scene_.maps()[idx].name)) != QMessageBox::Yes)
        return;

    bool wasActive = (idx == scene_.activeIndex());
    if (wasActive) clearViewerActiveMap();
    scene_.removeMapAt(idx);

    if (scene_.activeIndex() >= 0) {
        loadProjectFromActiveEntry();
        refreshActiveMapView();
    } else {
        project_ = MapProject();
        currentProjectDir_.clear();
    }
    refreshScenePanels();
    statusLabel_->setText("已移除地图");
}

void MainWindow::onRenameSlamMap(const QString& slamMapId, const QString& newName) {
    auto* e = scene_.mapById(slamMapId);
    if (!e) return;
    const QString nn = newName.trimmed();
    if (nn.isEmpty() || nn == e->name) return;
    e->name = nn;
    markDirty();
    refreshScenePanels();               // 刷新场景树 + 总览足迹标签
    statusLabel_->setText("已重命名地图为: " + nn);
}

void MainWindow::onExportSlamMap(const QString& slamMapId) {
    // 活跃地图取工作副本 project_（含未保存编辑），其余取条目内 project
    MapProject* proj = projectForMap(slamMapId);
    if (!proj || proj->submapIds().isEmpty()) {
        QMessageBox::warning(this, "导出失败", "该 SLAM 地图为空或不可用");
        return;
    }
    auto* e = scene_.mapById(slamMapId);
    const QString suggest = e ? (e->name.isEmpty() ? e->id : e->name) : slamMapId;

    QString parentDir = QFileDialog::getExistingDirectory(
        this, QString("选择导出「%1」的目标目录（将写入 <目录名>.json 与 PCD）").arg(suggest),
        currentProjectDir_,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (parentDir.isEmpty()) return;

    // 若选中的是一个非空的通用目录，则在其下新建一个以地图名命名的子目录，
    // 避免把 PCD 直接铺进用户的现有目录；若目录名就等于地图名则直接用它。
    QDir chosen(parentDir);
    QString targetDir = parentDir;
    if (chosen.dirName() != suggest) {
        targetDir = chosen.absoluteFilePath(suggest);
        if (QDir(targetDir).exists() && !QDir(targetDir).isEmpty()) {
            if (QMessageBox::question(this, "目录已存在",
                    QString("目录已存在且非空：\n%1\n是否覆盖其中同名文件？").arg(targetDir))
                != QMessageBox::Yes)
                return;
        }
    }

    statusLabel_->setText("导出中...");
    QApplication::processEvents();

    // 用副本导出，避免修改内存中该地图的 build_no
    MapProject copy = *proj;
    if (!copy.save(targetDir)) {
        QMessageBox::critical(this, "导出失败", "无法写入目录: " + targetDir);
        statusLabel_->setText("导出失败");
        return;
    }
    QMessageBox::information(this, "导出完成",
        QString("已将 SLAM 地图「%1」导出为独立工程:\n%2").arg(suggest, targetDir));
    statusLabel_->setText("已导出地图: " + suggest);
}

void MainWindow::onAlignSlamMap(const QString& slamMapId) {
    showLeftTab(kSceneTab);
    int i = sceneView_->indexOfItem(slamMapId);
    if (i >= 0) sceneView_->setSelected(i);
}

void MainWindow::onSlamMapActivated(const QString& slamMapId) {
    onSlamMapSelected(slamMapId);   // 切换活跃地图并刷新 3D
    showLeftTab(kSlamTab);          // 进入其内部 submap 编辑
}

void MainWindow::onOrthoVisibilityChanged(const QString& slamMapId, bool visible) {
    auto* e = scene_.mapById(slamMapId);
    if (e) e->orthoVisible = visible;
    sceneView_->setItemVisible(slamMapId, visible);
    markDirty();
}

// ── 场景地图 Tab：右侧属性栏（OSM 底图 / SLAM geo_anchor）─────────────
void MainWindow::onOsmSelected() {
    auto osm = scene_.osm();
    const QString src = scene_.hasOsm() ? QFileInfo(scene_.osmSource()).fileName()
                                        : QString();
    if (osm && osm->hasBounds())
        propsPanel_->showOsmProperties(true, osm->minLon(), osm->minLat(),
                                       osm->maxLon(), osm->maxLat(), src);
    else
        propsPanel_->showOsmProperties(false, 0, 0, 0, 0, src);
}

void MainWindow::showSlamGeoInPanel(const QString& slamMapId) {
    int i = sceneView_->indexOfItem(slamMapId);
    if (i >= 0) sceneView_->setSelected(i);
    auto* e = scene_.mapById(slamMapId);
    if (!e) return;
    // 若该地图尚无 anchor，用场景原点兜底展示
    GeoAnchor a = e->anchor;
    if (!a.valid && scene_.hasOrigin()) {
        a.lon = scene_.originLon();
        a.lat = scene_.originLat();
    }
    propsPanel_->showSlamGeoProperties(slamMapId, e->name.isEmpty() ? e->id : e->name, a);
}

void MainWindow::onSlamGeoAnchorEdited(const QString& slamMapId, const GeoAnchor& anchor) {
    auto* e = scene_.mapById(slamMapId);
    if (!e) return;
    e->anchor = anchor;
    int i = sceneView_->indexOfItem(slamMapId);
    if (i >= 0) sceneView_->updateAnchor(i, anchor);
    scenePanel_->rebuild(&scene_);
    markDirty();
}

// ── 场景语义点位（OSM 底图上，经纬度 + 航向角）───────────────────────
void MainWindow::onAddScenePoint() {
    if (!scene_.hasOrigin()) {
        QMessageBox::information(this, "新增语义点位",
            "请先导入 OSM/PBF 底图（用于确定经纬度基准）。");
        return;
    }
    scenePointPickForName_.clear();     // 空 = 新建
    showLeftTab(kSceneTab);
    sceneView_->setAddPointMode(true);
    statusLabel_->setText("请在底图上点选语义点位的位置");
}

void MainWindow::onScenePointSelected(const QString& name) {
    auto* p = scene_.osmPointByName(name);
    if (!p) return;
    selectedScenePoint_ = name;
    showLeftTab(kSceneTab);
    sceneView_->setSelectedScenePoint(name);
    propsPanel_->showSceneSemanticPoint(*p);
}

void MainWindow::onScenePointPlacedOnMap(double lon, double lat) {
    sceneView_->setAddPointMode(false);

    // 回填已选中点位的位置
    if (!scenePointPickForName_.isEmpty()) {
        if (auto* p = scene_.osmPointByName(scenePointPickForName_)) {
            p->lon = lon;
            p->lat = lat;
            markDirty();
            propsPanel_->setScenePointLonLat(lon, lat);
        }
        scenePointPickForName_.clear();
        refreshScenePanels();
        return;
    }

    // 新建点位
    SceneSemanticPoint p;
    p.name        = scene_.uniqueOsmPointName("point");
    p.type        = "single_pose";
    p.needInspect = false;
    p.lon         = lon;
    p.lat         = lat;
    p.headingDeg  = 0.0;
    scene_.osmPoints().append(p);
    selectedScenePoint_ = p.name;
    markDirty();
    refreshScenePanels();
    sceneView_->setSelectedScenePoint(p.name);
    propsPanel_->showSceneSemanticPoint(p);
    statusLabel_->setText("已新增语义点位: " + p.name);
}

void MainWindow::onScenePointMovedOnMap(const QString& name, double lon, double lat) {
    auto* p = scene_.osmPointByName(name);
    if (!p) return;
    p->lon = lon;
    p->lat = lat;
    selectedScenePoint_ = name;
    markDirty();
    propsPanel_->setScenePointLonLat(lon, lat);
    // 树上的坐标提示同步更新（不影响 sceneView，其内部已实时更新）
    scenePanel_->rebuild(&scene_);
}

void MainWindow::onSceneSemanticPointApply(const QString& oldName, const SceneSemanticPoint& pt) {
    const QString nn = pt.name.trimmed();
    if (nn.isEmpty()) return;

    // 重名校验（排除自身）
    for (const auto& q : scene_.osmPoints()) {
        if (q.name == nn && nn != oldName) {
            QMessageBox::warning(this, "场景语义点位",
                QString("名称 \"%1\" 已存在。").arg(nn));
            return;
        }
    }

    SceneSemanticPoint* p = scene_.osmPointByName(oldName);
    if (!p) p = scene_.osmPointByName(nn);
    if (!p) return;

    *p = pt;
    p->name = nn;
    selectedScenePoint_ = nn;
    markDirty();
    refreshScenePanels();
    sceneView_->setSelectedScenePoint(nn);
    statusLabel_->setText("已更新语义点位: " + nn);
}

void MainWindow::onRemoveScenePoint(const QString& name) {
    auto& pts = scene_.osmPoints();
    for (int i = 0; i < pts.size(); ++i) {
        if (pts[i].name == name) {
            pts.remove(i);
            break;
        }
    }
    if (selectedScenePoint_ == name) {
        selectedScenePoint_.clear();
        sceneView_->setSelectedScenePoint(QString());
        propsPanel_->clear();
    }
    markDirty();
    refreshScenePanels();
    statusLabel_->setText("已删除语义点位: " + name);
}

void MainWindow::onRenameScenePoint(const QString& oldName, const QString& newName) {
    const QString nn = newName.trimmed();
    if (nn.isEmpty() || nn == oldName) return;
    for (const auto& q : scene_.osmPoints()) {
        if (q.name == nn) {
            QMessageBox::warning(this, "场景语义点位",
                QString("名称 \"%1\" 已存在。").arg(nn));
            return;
        }
    }
    auto* p = scene_.osmPointByName(oldName);
    if (!p) return;
    p->name = nn;
    if (selectedScenePoint_ == oldName) selectedScenePoint_ = nn;
    markDirty();
    refreshScenePanels();
    if (auto* sel = scene_.osmPointByName(nn))
        propsPanel_->showSceneSemanticPoint(*sel);
}

// ═══════════════════════════════════════════════════════════════════════
//  正交叠加对齐窗口：各 SLAM 地图俯视投影叠加到 OSM 底图
// ═══════════════════════════════════════════════════════════════════════

static CloudPtr mergeProjectRootCloud(MapProject* proj) {
    CloudPtr merged(new CloudT);
    if (!proj) return merged;
    for (const auto& id : proj->submapIds()) {
        auto* sm = proj->submap(id);
        if (!sm || !sm->cloud() || sm->cloud()->empty()) continue;
        CloudT tmp;
        pcl::transformPointCloud(*sm->cloud(), tmp,
                                 proj->submapToRootMatrix(id).cast<float>());
        *merged += tmp;
    }
    return merged;
}

void MainWindow::rebuildSceneOverview() {
    syncActiveEntryFromProject();
    sceneView_->clearItems();
    sceneViewRowToId_.clear();

    double oLon = scene_.hasOrigin() ? scene_.originLon() : 0.0;
    double oLat = scene_.hasOrigin() ? scene_.originLat() : 0.0;
    sceneView_->setOsm(scene_.osm(), oLon, oLat);

    static const QVector<QColor> palette = {
        QColor(255,130,80), QColor(120,220,120), QColor(120,170,255),
        QColor(220,130,220), QColor(255,220,80), QColor(80,220,210)};
    for (int i = 0; i < scene_.maps().size(); ++i) {
        auto& e = scene_.maps()[i];
        CloudPtr cloud = (e.id == activeMapId()) ? mergeProjectRootCloud(&project_)
                                                 : mergeProjectRootCloud(e.project.get());
        if (!cloud || cloud->empty()) continue;
        GeoAnchor a = e.anchor;
        if (!a.valid && scene_.hasOrigin()) { a.lon = oLon; a.lat = oLat; a.valid = true; }
        sceneView_->addSlamItem(e.id, e.name, cloud, palette[i % palette.size()], a);
        sceneView_->setItemVisible(e.id, e.orthoVisible);
        sceneViewRowToId_.append(e.id);
    }
    sceneView_->setScenePoints(scene_.osmPoints());
    sceneView_->setSelectedScenePoint(selectedScenePoint_);
    sceneView_->setSelected(-1);
    sceneView_->fitView();
    sceneOverviewDirty_ = false;
}

void MainWindow::ensureSceneOverview() {
    if (sceneOverviewDirty_) rebuildSceneOverview();
}

void MainWindow::showLeftTab(int index) {
    if (leftTabs_->currentIndex() != index)
        leftTabs_->setCurrentIndex(index);   // 触发 onLeftTabChanged
    else
        onLeftTabChanged(index);
}

void MainWindow::onLeftTabChanged(int index) {
    if (index == kSceneTab) {
        ensureSceneOverview();
        centralStack_->setCurrentWidget(sceneView_);
    } else {
        centralStack_->setCurrentWidget(viewer_);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  场景：打开 / 保存（单文件 scene.json）
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onOpenScene() {
    if (!maybePromptSaveChanges()) return;
    QString dir = QFileDialog::getExistingDirectory(
        this, "打开场景目录（含 scene.json）", currentProjectDir_,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (dir.isEmpty()) return;

    clearViewerActiveMap();
    if (!scene_.loadFromDir(dir)) {
        QMessageBox::critical(this, "错误",
                              "无法加载场景（目录中缺少 scene.json？）: " + dir);
        return;
    }
    if (scene_.activeMap()) {
        loadProjectFromActiveEntry();
        colorIndex_ = 0;
        for (const auto& id : project_.submapIds()) {
            auto* sm = project_.submap(id);
            if (sm) sm->displayOptions().color = nextColor();
        }
    } else {
        project_ = MapProject();
    }
    currentProjectDir_ = dir;
    refreshScenePanels();
    refreshActiveMapView();
    viewer_->resetCamera();
    showLeftTab(scene_.hasOsm() ? kSceneTab : kSlamTab);
    project_.setDirty(false);
    updateWindowTitle();
    statusLabel_->setText(QString("已加载场景: %1  (%2 张地图)")
                              .arg(QFileInfo(dir).fileName()).arg(scene_.mapCount()));
}

void MainWindow::onSaveScene() {
    // 与「另存为」一致：统一保存到单个 scene.json
    onSaveAs();
}

// ═══════════════════════════════════════════════════════════════════════
//  全局路线
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onAddGlobalRoute() {
    bool ok = false;
    QString name = QInputDialog::getText(this, "新增全局路线",
        "全局路线名称:", QLineEdit::Normal, "", &ok);
    name = name.trimmed();
    if (!ok || name.isEmpty()) return;
    for (const auto& r : scene_.globalRoutes())
        if (r.name == name) { QMessageBox::warning(this, "提示", "同名路线已存在"); return; }
    GlobalRoute r; r.name = name;
    scene_.globalRoutes().append(r);
    scenePanel_->rebuild(&scene_);
    markDirty();
}

void MainWindow::onRemoveGlobalRoute(const QString& name) {
    auto& rs = scene_.globalRoutes();
    for (int i = 0; i < rs.size(); ++i)
        if (rs[i].name == name) { rs.remove(i); break; }
    scenePanel_->rebuild(&scene_);
    markDirty();
}

void MainWindow::onRenameGlobalRoute(const QString& oldName, const QString& newName) {
    for (auto& r : scene_.globalRoutes())
        if (r.name == oldName) { r.name = newName; break; }
    scenePanel_->rebuild(&scene_);
    markDirty();
}

void MainWindow::onAddGlobalWaypoint(const QString& routeName) {
    GlobalRoute* route = nullptr;
    for (auto& r : scene_.globalRoutes())
        if (r.name == routeName) { route = &r; break; }
    if (!route) return;

    QStringList kinds{"地图点位 (slam_map/submap/semantic_point)", "GPS 经纬度", "指令 (instruction)"};
    bool ok = false;
    QString kind = QInputDialog::getItem(this, "途经点类型", "选择途经点类型:", kinds, 0, false, &ok);
    if (!ok) return;

    GlobalWaypoint wp;
    if (kind == kinds[0]) {
        wp.kind = GlobalWaypoint::Map;
        QStringList mapIds;
        for (const auto& e : scene_.maps()) mapIds << e.id;
        if (mapIds.isEmpty()) { QMessageBox::warning(this, "提示", "场景中还没有 SLAM 地图"); return; }
        wp.slamMap = QInputDialog::getItem(this, "选择地图", "slam_map:", mapIds, 0, false, &ok);
        if (!ok) return;
        MapProject* proj = projectForMap(wp.slamMap);
        QStringList subs = proj ? proj->submapIds() : QStringList();
        wp.submap = QInputDialog::getItem(this, "选择子地图", "submap:", subs, 0, true, &ok);
        if (!ok) return;
        wp.semanticPoint = QInputDialog::getText(this, "语义点", "semantic_point:",
                                                 QLineEdit::Normal, "", &ok);
        if (!ok) return;
    } else if (kind == kinds[1]) {
        wp.kind = GlobalWaypoint::Gps;
        wp.lon = QInputDialog::getDouble(this, "GPS", "经度 lon:", 0, -180, 180, 8, &ok);
        if (!ok) return;
        wp.lat = QInputDialog::getDouble(this, "GPS", "纬度 lat:", 0, -90, 90, 8, &ok);
        if (!ok) return;
    } else {
        wp.kind = GlobalWaypoint::Instruction;
        wp.instruction = QInputDialog::getText(this, "指令", "instruction:",
                                               QLineEdit::Normal, "", &ok);
        if (!ok || wp.instruction.isEmpty()) return;
    }
    route->waypoints.append(wp);
    scenePanel_->rebuild(&scene_);
    markDirty();
}

void MainWindow::onRemoveGlobalWaypoint(const QString& routeName, int index) {
    for (auto& r : scene_.globalRoutes()) {
        if (r.name == routeName) {
            if (index >= 0 && index < r.waypoints.size()) r.waypoints.remove(index);
            break;
        }
    }
    scenePanel_->rebuild(&scene_);
    markDirty();
}

// ═══════════════════════════════════════════════════════════════════════
//  内部路线（属于某张 SLAM 地图）
// ═══════════════════════════════════════════════════════════════════════

void MainWindow::onAddInternalRoute(const QString& mapId) {
    MapProject* proj = projectForMap(mapId);
    if (!proj) return;
    bool ok = false;
    QString name = QInputDialog::getText(this, "新增内部路线",
        "路线名称:", QLineEdit::Normal, "", &ok);
    name = name.trimmed();
    if (!ok || name.isEmpty()) return;
    if (proj->routes().contains(name)) { QMessageBox::warning(this, "提示", "同名路线已存在"); return; }
    Route r; r.name = name;
    proj->addRoute(r);
    if (mapId == activeMapId()) markDirty();
    refreshScenePanels();
}

void MainWindow::onRemoveInternalRoute(const QString& mapId, const QString& routeName) {
    MapProject* proj = projectForMap(mapId);
    if (!proj) return;
    proj->removeRoute(routeName);
    if (mapId == activeMapId()) { markDirty(); refreshRouteLines(); }
    refreshScenePanels();
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

    // 若该 submap 的 VPR 紫色点已加载，同步把世界变换跟上（对齐 transform
    // 改变后 VPR 点应跟着点云一起动）
    if (vprDbs_.contains(id)) {
        viewer_->setVprTransform(id, vprWorldTransform(id));
    }
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

    // 全程使用 const 引用做查找：project_ 的 routes_/submaps_ 在导入其它 SLAM 地图后
    // 可能与场景条目里的 MapProject 处于隐式共享(COW)状态；若在遍历 routes() 的同时
    // 调用非 const 的 route()/submap()，QMap 会 detach 并重新分配，导致外层迭代器失效
    // → 读到已释放的内存而崩溃。const 查找不会触发 detach。
    const MapProject& cproj = project_;

    // 递归展开嵌套路线引用为扁平 waypoint 列表
    std::function<QVector<RouteWaypoint>(const Route&, QSet<QString>&)> flattenRoute =
        [&](const Route& route, QSet<QString>& visited) -> QVector<RouteWaypoint> {
        QVector<RouteWaypoint> flat;
        for (const auto& wp : route.waypoints) {
            if (wp.isRouteRef()) {
                if (visited.contains(wp.routeRef)) continue;  // 防止循环引用
                const auto* nested = cproj.route(wp.routeRef);
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

    for (auto it = cproj.routes().begin(); it != cproj.routes().end(); ++it) {
        QSet<QString> visited;
        visited.insert(it.key());
        auto flatWaypoints = flattenRoute(it.value(), visited);
        QVector<MarkerVertex> lineVerts;

        for (const auto& wp : flatWaypoints) {
            const auto* sm = cproj.submap(wp.submapId);
            if (!sm) continue;
            const auto* sp = sm->semanticPoint(wp.semanticPoint);
            if (!sp || sp->poses.isEmpty()) continue;

            Eigen::Matrix4d T = cproj.submapToRootMatrix(wp.submapId);
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
        // 数据库变了：清缓存 + 清紫色点 + 清选中。下次勾选 VPR 视觉特征时重新加载
        vprDbs_.remove(smId);
        viewer_->removeVprPoints(smId);
        for (auto it = vprSelection_.begin(); it != vprSelection_.end(); ) {
            if (it->first == smId) it = vprSelection_.erase(it);
            else ++it;
        }
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

// ═══════════════════════════════════════════════════════════════════════
//  VPR 视觉特征点
// ═══════════════════════════════════════════════════════════════════════

Eigen::Matrix4d MainWindow::vprWorldTransform(const QString& smId) const {
    // 与点云一致：submap -> root（默认 submap）的级联变换
    return project_.submapToRootMatrix(smId);
}

std::shared_ptr<VprDatabaseData> MainWindow::ensureVprDatabase(const QString& smId) {
    auto it = vprDbs_.find(smId);
    if (it != vprDbs_.end()) return it.value();
    if (currentProjectDir_.isEmpty()) return nullptr;
    QString dbDir = QDir(currentProjectDir_).absoluteFilePath("vpr/" + smId);
    if (!VprDatabaseData::dirHasDatabase(dbDir)) return nullptr;
    auto db = std::make_shared<VprDatabaseData>();
    QString err;
    if (!db->load(dbDir, &err)) {
        QMessageBox::warning(this, "VPR 数据库加载失败",
                             QString("Submap「%1」: %2").arg(smId, err));
        return nullptr;
    }
    vprDbs_[smId] = db;
    return db;
}

void MainWindow::pushVprPointsToViewer(const QString& smId) {
    auto db = vprDbs_.value(smId);
    if (!db || !db->isLoaded()) return;
    QVector<Eigen::Vector3d> pts;
    pts.reserve(db->size());
    for (const auto& e : db->entries()) pts.append(e.xyz);
    viewer_->updateVprPoints(smId, pts, vprWorldTransform(smId));
}

void MainWindow::onShowVprFeaturesChanged(const QString& smId, bool show) {
    if (auto* sm = project_.submap(smId)) {
        sm->displayOptions().showVprFeatures = show;
    }
    if (show) {
        auto db = ensureVprDatabase(smId);
        if (!db) return;                       // 加载失败 / 无数据库
        pushVprPointsToViewer(smId);
        viewer_->setVprPointsVisible(smId, true);
        statusLabel_->setText(QString("已显示 VPR 视觉特征: %1 (%2 点)")
                                  .arg(smId).arg(db->size()));
    } else {
        viewer_->setVprPointsVisible(smId, false);
    }
}

void MainWindow::onVprPointClicked(const QString& smId, int index) {
    auto db = vprDbs_.value(smId);
    if (!db || !db->isLoaded()) return;
    if (index < 0 || index >= db->size()) return;
    QString imgPath = db->resolveImagePath(index);
    if (imgPath.isEmpty()) {
        QMessageBox::information(this, "VPR",
            QString("该点没有对应的图片路径（%1[%2]）").arg(smId).arg(index));
        return;
    }
    if (!QFileInfo::exists(imgPath)) {
        QMessageBox::warning(this, "VPR",
            QString("图片文件不存在:\n%1").arg(imgPath));
        return;
    }
    // 非模态、自删除：用户可同时打开多张比较
    auto* dlg = new VprImageViewer(smId, index, imgPath, this);
    dlg->show();
}

void MainWindow::onVprSelectionChanged(const QSet<QPair<QString, int>>& selected) {
    vprSelection_ = selected;
    statusLabel_->setText(
        selected.isEmpty()
            ? QStringLiteral("VPR 选择已清空")
            : QString("已选中 %1 个 VPR 视觉特征").arg(selected.size()));
}

void MainWindow::onDeleteSelectedVprFeaturesRequested(const QString& smId) {
    // 只删该 submap 中被选中的；用户右键的是某 submap 节点，跨 submap 选择
    // 时仅处理该 submap 的部分
    QVector<int> indices;
    for (const auto& p : vprSelection_) {
        if (p.first == smId) indices.append(p.second);
    }
    if (indices.isEmpty()) {
        QMessageBox::information(this, "提示",
            QString("Submap「%1」当前没有选中的 VPR 视觉特征。\n"
                    "请先按住 Shift + 左键在 3D 视图中拖框选择。").arg(smId));
        return;
    }
    auto db = ensureVprDatabase(smId);
    if (!db) return;
    if (indices.size() >= db->size()) {
        QMessageBox::warning(this, "不允许",
            QStringLiteral("不能一次性删除一个 submap 的全部 VPR 视觉特征；\n"
                           "如需重置，请使用「构建 VPR 数据库」重新生成。"));
        return;
    }
    auto ret = QMessageBox::question(
        this, "确认删除",
        QString("将从「%1」的 VPR 数据库中永久删除 %2 个特征点，\n"
                "并同步更新磁盘上的 descriptors / poses / image_paths 三个文件。\n\n"
                "继续？").arg(smId).arg(indices.size()),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (ret != QMessageBox::Yes) return;

    QString err;
    if (!db->removeIndices(indices, &err)) {
        QMessageBox::critical(this, "删除失败", err);
        return;
    }
    // 内存 + 视图同步
    vprSelection_.clear();
    pushVprPointsToViewer(smId);
    viewer_->setVprPointsVisible(smId, true);
    viewer_->clearVprSelection();
    statusLabel_->setText(QString("已删除 %1 个 VPR 视觉特征，剩余 %2 个")
                              .arg(indices.size()).arg(db->size()));
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
