#include "ui/SceneAlignDialog.h"
#include "ui/SceneAlignView.h"
#include "core/MapProject.h"
#include "core/Wgs84.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QListWidget>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileInfo>
#include <QDir>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

namespace me {

SceneAlignDialog::SceneAlignDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle("场景对齐 · OSM 底图 + SLAM 地图");
    setWindowFlag(Qt::Window, true);   // 独立窗口(可最大化)
    resize(1200, 780);
    setupUi();
    setEditorsEnabled(false);
}

// ═══════════════════════════════════════════════════════════════════════
//  UI
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignDialog::setupUi() {
    auto* root = new QHBoxLayout(this);

    view_ = new SceneAlignView(this);
    root->addWidget(view_, 1);

    // ── 右侧面板 ─────────────────────────────────────────────────────
    auto* side = new QVBoxLayout;
    root->addLayout(side, 0);

    // 文件操作
    auto* fileBox = new QGroupBox("场景文件", this);
    auto* fileLay = new QVBoxLayout(fileBox);
    auto* openBtn = new QPushButton("打开场景…", this);
    auto* saveBtn = new QPushButton("保存场景…", this);
    fileLay->addWidget(openBtn);
    fileLay->addWidget(saveBtn);
    auto* nameRow = new QHBoxLayout;
    nameRow->addWidget(new QLabel("名称:", this));
    sceneNameEdit_ = new QLineEdit(scene_.name(), this);
    nameRow->addWidget(sceneNameEdit_);
    fileLay->addLayout(nameRow);
    side->addWidget(fileBox);

    // 底图
    auto* osmBox = new QGroupBox("OSM 底图", this);
    auto* osmLay = new QVBoxLayout(osmBox);
    auto* importOsmBtn = new QPushButton("导入 OSM (.osm / .pbf)…", this);
    osmLabel_ = new QLabel("未导入", this);
    osmLabel_->setWordWrap(true);
    originLabel_ = new QLabel("原点: —", this);
    originLabel_->setWordWrap(true);
    osmLay->addWidget(importOsmBtn);
    osmLay->addWidget(osmLabel_);
    osmLay->addWidget(originLabel_);
    side->addWidget(osmBox);

    // SLAM 列表
    auto* slamBox = new QGroupBox("SLAM 地图", this);
    auto* slamLay = new QVBoxLayout(slamBox);
    auto* importSlamBtn = new QPushButton("导入 SLAM 地图…", this);
    list_ = new QListWidget(this);
    removeBtn_ = new QPushButton("移除选中", this);
    slamLay->addWidget(importSlamBtn);
    slamLay->addWidget(list_);
    slamLay->addWidget(removeBtn_);
    side->addWidget(slamBox, 1);

    // geo_anchor 编辑
    auto* anchorBox = new QGroupBox("对齐 (geo_anchor)", this);
    auto* form = new QFormLayout(anchorBox);
    auto mkSpin = [this](double lo, double hi, int dec, double step) {
        auto* s = new QDoubleSpinBox(this);
        s->setRange(lo, hi);
        s->setDecimals(dec);
        s->setSingleStep(step);
        s->setKeyboardTracking(false);
        return s;
    };
    lonSpin_     = mkSpin(-180.0, 180.0, 8, 0.00001);
    latSpin_     = mkSpin(-90.0, 90.0, 8, 0.00001);
    altSpin_     = mkSpin(-1000.0, 10000.0, 3, 0.1);
    headingSpin_ = mkSpin(-180.0, 180.0, 2, 1.0);
    headingSpin_->setWrapping(true);
    scaleSpin_   = mkSpin(0.05, 20.0, 4, 0.01);
    form->addRow("经度 lon:", lonSpin_);
    form->addRow("纬度 lat:", latSpin_);
    form->addRow("海拔 alt:", altSpin_);
    form->addRow("朝向 heading(°):", headingSpin_);
    form->addRow("缩放 scale:", scaleSpin_);
    auto* fitBtn = new QPushButton("适配视图", this);
    form->addRow(fitBtn);
    side->addWidget(anchorBox);

    // 连接
    connect(openBtn, &QPushButton::clicked, this, &SceneAlignDialog::onOpenScene);
    connect(saveBtn, &QPushButton::clicked, this, &SceneAlignDialog::onSaveScene);
    connect(importOsmBtn, &QPushButton::clicked, this, &SceneAlignDialog::onImportOsm);
    connect(importSlamBtn, &QPushButton::clicked, this, &SceneAlignDialog::onImportSlam);
    connect(removeBtn_, &QPushButton::clicked, this, &SceneAlignDialog::onRemoveSlam);
    connect(fitBtn, &QPushButton::clicked, this, &SceneAlignDialog::onFitView);
    connect(list_, &QListWidget::currentRowChanged, this, &SceneAlignDialog::onListRowChanged);
    connect(view_, &SceneAlignView::selectionChanged, this, &SceneAlignDialog::onViewSelectionChanged);
    connect(view_, &SceneAlignView::anchorEdited, this, &SceneAlignDialog::onViewAnchorEdited);
    connect(sceneNameEdit_, &QLineEdit::textChanged, this,
            [this](const QString& t) { scene_.setName(t); });
    for (auto* s : {lonSpin_, latSpin_, altSpin_, headingSpin_, scaleSpin_})
        connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &SceneAlignDialog::onAnchorSpinChanged);
}

QColor SceneAlignDialog::colorForIndex(int i) const {
    static const QVector<QColor> palette = {
        QColor(230, 120, 120), QColor(120, 200, 140), QColor(120, 160, 230),
        QColor(230, 200, 110), QColor(200, 130, 220), QColor(120, 210, 210),
        QColor(230, 160, 110), QColor(170, 190, 120)};
    return palette[((i % palette.size()) + palette.size()) % palette.size()];
}

void SceneAlignDialog::setEditorsEnabled(bool on) {
    for (auto* s : {lonSpin_, latSpin_, altSpin_, headingSpin_, scaleSpin_})
        if (s) s->setEnabled(on);
    if (removeBtn_) removeBtn_->setEnabled(on);
}

// ═══════════════════════════════════════════════════════════════════════
//  点云载入
// ═══════════════════════════════════════════════════════════════════════
CloudPtr SceneAlignDialog::loadCloudFromSource(const QString& path, QString& outName) {
    QFileInfo fi(path);
    outName = fi.completeBaseName();

    if (fi.isFile() && fi.suffix().toLower() == "pcd") {
        auto cloud = std::make_shared<CloudT>();
        if (pcl::io::loadPCDFile<PointT>(path.toStdString(), *cloud) == -1)
            return nullptr;
        return cloud;
    }

    // 目录 or .json → 作为多 submap 工程加载并合并到 root 系
    QString projDir = fi.isDir() ? fi.absoluteFilePath() : fi.absolutePath();
    outName = QFileInfo(projDir).completeBaseName();

    MapProject proj;
    if (!proj.load(projDir)) return nullptr;

    auto merged = std::make_shared<CloudT>();
    for (const QString& id : proj.submapIds()) {
        const SubMap* sm = proj.submap(id);
        if (!sm || !sm->cloud() || sm->cloud()->empty()) continue;
        Eigen::Matrix4f m = proj.submapToRootMatrix(id).cast<float>();
        CloudT transformed;
        pcl::transformPointCloud(*sm->cloud(), transformed, m);
        *merged += transformed;
    }
    if (merged->empty()) return nullptr;
    return merged;
}

// ═══════════════════════════════════════════════════════════════════════
//  导入 OSM
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignDialog::onImportOsm() {
    QString path = QFileDialog::getOpenFileName(
        this, "选择 OSM 底图", QString(),
        "OSM 地图 (*.osm *.pbf *.osm.pbf);;OSM XML (*.osm);;OSM PBF (*.pbf *.osm.pbf);;所有文件 (*)");
    if (path.isEmpty()) return;

    auto osm = std::make_shared<OsmData>();
    if (!osm->load(path) || osm->isEmpty()) {
        QMessageBox::critical(this, "错误", "无法解析 OSM 文件或文件为空:\n" + path);
        return;
    }

    osm_ = osm;
    absOsmSource_ = path;
    scene_.setOsmSource(path);
    if (osm_->hasBounds()) {
        scene_.setBounds(osm_->minLon(), osm_->minLat(), osm_->maxLon(), osm_->maxLat());
        // 若尚无原点，用底图中心
        if (!scene_.hasOrigin())
            scene_.setOrigin(osm_->centerLon(), osm_->centerLat());
    }

    view_->setOsm(osm_, scene_.originLon(), scene_.originLat());
    osmLabel_->setText(QString("%1\nnodes=%2 ways=%3")
                           .arg(QFileInfo(path).fileName())
                           .arg(osm_->nodes().size())
                           .arg(osm_->ways().size()));
    originLabel_->setText(QString("原点: lon=%1  lat=%2")
                              .arg(scene_.originLon(), 0, 'f', 7)
                              .arg(scene_.originLat(), 0, 'f', 7));
    view_->fitView();
}

// ═══════════════════════════════════════════════════════════════════════
//  导入 SLAM 地图
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignDialog::onImportSlam() {
    // 允许选目录(工程)或 .pcd 文件；用非原生对话框以同时支持
    QString path = QFileDialog::getExistingDirectory(
        this, "选择 SLAM 地图工程目录 (取消可改选 .pcd)", QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (path.isEmpty()) {
        path = QFileDialog::getOpenFileName(
            this, "选择 SLAM 点云 (.pcd)", QString(), "PCD 点云 (*.pcd)");
        if (path.isEmpty()) return;
    }

    QString name;
    CloudPtr cloud = loadCloudFromSource(path, name);
    if (!cloud || cloud->empty()) {
        QMessageBox::critical(this, "错误", "无法从以下位置载入点云:\n" + path);
        return;
    }

    // 若还没有场景原点：以 (0,0) 起步(等 OSM 导入再更新)
    if (!scene_.hasOrigin()) {
        scene_.setOrigin(0.0, 0.0);
        view_->setOrigin(0.0, 0.0);
        originLabel_->setText("原点: (0, 0) — 建议先导入 OSM 底图");
    }

    SlamMapRef ref;
    ref.id   = scene_.uniqueId(name);
    ref.name = name;
    ref.source = path;
    ref.anchor.lon = scene_.originLon();
    ref.anchor.lat = scene_.originLat();
    ref.anchor.alt = 0.0;
    ref.anchor.headingDeg = 0.0;
    ref.anchor.scale = 1.0;
    ref.anchor.valid = true;

    const int idx = scene_.slamMaps().size();
    scene_.addSlamMap(ref);
    absSlamSources_.append(path);

    view_->addSlamItem(ref.id, ref.name, cloud, colorForIndex(idx), ref.anchor);
    rebuildList();
    list_->setCurrentRow(idx);
    view_->fitView();
}

void SceneAlignDialog::onRemoveSlam() {
    int row = list_->currentRow();
    if (row < 0 || row >= scene_.slamMaps().size()) return;
    scene_.removeSlamMapAt(row);
    if (row < absSlamSources_.size()) absSlamSources_.remove(row);
    view_->removeItem(row);
    rebuildList();
}

// ═══════════════════════════════════════════════════════════════════════
//  列表 / 选择同步
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignDialog::rebuildList() {
    const int keep = list_->currentRow();
    list_->blockSignals(true);
    list_->clear();
    const auto& maps = scene_.slamMaps();
    for (int i = 0; i < maps.size(); ++i) {
        auto* item = new QListWidgetItem(
            QString("%1  (%2)").arg(maps[i].name, maps[i].id));
        item->setForeground(colorForIndex(i));
        list_->addItem(item);
    }
    list_->blockSignals(false);
    if (keep >= 0 && keep < maps.size()) list_->setCurrentRow(keep);
    setEditorsEnabled(!maps.isEmpty());
}

void SceneAlignDialog::onListRowChanged(int row) {
    view_->setSelected(row);
    syncAnchorEditors(row);
}

void SceneAlignDialog::onViewSelectionChanged(int index) {
    if (list_->currentRow() != index) {
        list_->blockSignals(true);
        list_->setCurrentRow(index);
        list_->blockSignals(false);
    }
    syncAnchorEditors(index);
}

void SceneAlignDialog::onViewAnchorEdited(int index, const GeoAnchor& anchor) {
    if (index < 0 || index >= scene_.slamMaps().size()) return;
    scene_.slamMaps()[index].anchor = anchor;
    if (index == list_->currentRow()) syncAnchorEditors(index);
}

void SceneAlignDialog::syncAnchorEditors(int index) {
    if (index < 0 || index >= scene_.slamMaps().size()) { setEditorsEnabled(false); return; }
    setEditorsEnabled(true);
    const GeoAnchor& a = scene_.slamMaps()[index].anchor;
    syncingEditors_ = true;
    lonSpin_->setValue(a.lon);
    latSpin_->setValue(a.lat);
    altSpin_->setValue(a.alt);
    headingSpin_->setValue(a.headingDeg);
    scaleSpin_->setValue(a.scale);
    syncingEditors_ = false;
}

void SceneAlignDialog::onAnchorSpinChanged() {
    if (syncingEditors_) return;
    int row = list_->currentRow();
    if (row < 0 || row >= scene_.slamMaps().size()) return;
    GeoAnchor a = scene_.slamMaps()[row].anchor;
    a.lon = lonSpin_->value();
    a.lat = latSpin_->value();
    a.alt = altSpin_->value();
    a.headingDeg = headingSpin_->value();
    a.scale = scaleSpin_->value();
    a.valid = true;
    scene_.slamMaps()[row].anchor = a;
    view_->updateAnchor(row, a);
}

void SceneAlignDialog::onFitView() { view_->fitView(); }

// ═══════════════════════════════════════════════════════════════════════
//  保存 / 打开 场景
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignDialog::onSaveScene() {
    QString path = sceneJsonPath_;
    if (path.isEmpty()) {
        path = QFileDialog::getSaveFileName(
            this, "保存场景 JSON", "scene.json", "场景 (*.json)");
        if (path.isEmpty()) return;
    }
    QDir sceneDir = QFileInfo(path).absoluteDir();

    // 把绝对路径转为相对场景目录（可移植）
    if (!absOsmSource_.isEmpty())
        scene_.setOsmSource(sceneDir.relativeFilePath(absOsmSource_));
    for (int i = 0; i < scene_.slamMaps().size() && i < absSlamSources_.size(); ++i)
        scene_.slamMaps()[i].source = sceneDir.relativeFilePath(absSlamSources_[i]);

    if (!scene_.save(path)) {
        QMessageBox::critical(this, "错误", "保存失败:\n" + path);
        return;
    }
    sceneJsonPath_ = path;
    setWindowTitle("场景对齐 · " + QFileInfo(path).fileName());
    QMessageBox::information(this, "已保存", "场景已保存到:\n" + path);
}

void SceneAlignDialog::onOpenScene() {
    QString path = QFileDialog::getOpenFileName(
        this, "打开场景 JSON", QString(), "场景 (*.json);;所有文件 (*)");
    if (path.isEmpty()) return;

    SceneModel loaded;
    if (!loaded.load(path)) {
        QMessageBox::critical(this, "错误", "无法加载场景:\n" + path);
        return;
    }

    // 重置状态
    scene_ = loaded;
    sceneJsonPath_ = path;
    osm_.reset();
    absOsmSource_.clear();
    absSlamSources_.clear();
    view_->clearItems();
    QDir sceneDir(loaded.sceneDir());

    sceneNameEdit_->setText(scene_.name());

    // OSM
    if (!scene_.osmSource().isEmpty()) {
        QString osmAbs = QFileInfo(scene_.osmSource()).isAbsolute()
                             ? scene_.osmSource()
                             : sceneDir.absoluteFilePath(scene_.osmSource());
        auto osm = std::make_shared<OsmData>();
        if (osm->load(osmAbs) && !osm->isEmpty()) {
            osm_ = osm;
            absOsmSource_ = osmAbs;
            if (!scene_.hasOrigin() && osm_->hasBounds())
                scene_.setOrigin(osm_->centerLon(), osm_->centerLat());
            view_->setOsm(osm_, scene_.originLon(), scene_.originLat());
            osmLabel_->setText(QString("%1\nnodes=%2 ways=%3")
                                   .arg(QFileInfo(osmAbs).fileName())
                                   .arg(osm_->nodes().size())
                                   .arg(osm_->ways().size()));
        } else {
            osmLabel_->setText("底图加载失败: " + scene_.osmSource());
        }
    }
    if (scene_.hasOrigin()) {
        view_->setOrigin(scene_.originLon(), scene_.originLat());
        originLabel_->setText(QString("原点: lon=%1  lat=%2")
                                  .arg(scene_.originLon(), 0, 'f', 7)
                                  .arg(scene_.originLat(), 0, 'f', 7));
    }

    // SLAM 地图
    for (int i = 0; i < scene_.slamMaps().size(); ++i) {
        auto& ref = scene_.slamMaps()[i];
        QString srcAbs = QFileInfo(ref.source).isAbsolute()
                             ? ref.source
                             : sceneDir.absoluteFilePath(ref.source);
        absSlamSources_.append(srcAbs);
        QString nm;
        CloudPtr cloud = loadCloudFromSource(srcAbs, nm);
        view_->addSlamItem(ref.id, ref.name, cloud, colorForIndex(i), ref.anchor);
        if (!cloud)
            qWarning() << "SLAM 源载入失败:" << srcAbs;
    }

    rebuildList();
    setWindowTitle("场景对齐 · " + QFileInfo(path).fileName());
    view_->fitView();
}

} // namespace me
