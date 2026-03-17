#include "ui/PropertiesPanel.h"

#include <QVBoxLayout>
#include <QScrollArea>

namespace me {

PropertiesPanel::PropertiesPanel(QWidget* parent) : QWidget(parent) {
    setupUI();
}

QDoubleSpinBox* PropertiesPanel::makeSpinBox(double min, double max, int decimals) {
    auto* sb = new QDoubleSpinBox();
    sb->setRange(min, max);
    sb->setDecimals(decimals);
    sb->setSingleStep(0.1);
    return sb;
}

void PropertiesPanel::setupUI() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    stack_ = new QStackedWidget();

    // 空白占位页
    auto* emptyPage = new QWidget();
    auto* emptyLayout = new QVBoxLayout(emptyPage);
    emptyLayout->addWidget(new QLabel("选择左侧项目以编辑属性"));
    emptyLayout->addStretch();
    stack_->addWidget(emptyPage);  // index 0

    setupSubmapPage();       // index 1
    setupSemanticPointPage();// index 2
    setupPortalPage();       // index 3

    auto* scroll = new QScrollArea();
    scroll->setWidget(stack_);
    scroll->setWidgetResizable(true);
    mainLayout->addWidget(scroll);
}

// ═══════════════════════════════════════════════════════════════════════
//  Submap 属性页
// ═══════════════════════════════════════════════════════════════════════

void PropertiesPanel::setupSubmapPage() {
    smPage_ = new QWidget();
    auto* layout = new QVBoxLayout(smPage_);

    // 基本信息
    auto* infoGroup = new QGroupBox("基本信息");
    auto* infoForm = new QFormLayout(infoGroup);
    smId_      = new QLineEdit(); smId_->setReadOnly(true);
    smName_    = new QLineEdit();
    smDesc_    = new QLineEdit();
    smMapFile_ = new QLineEdit(); smMapFile_->setReadOnly(true);
    smCloudInfo_ = new QLabel();
    infoForm->addRow("ID:", smId_);
    infoForm->addRow("名称:", smName_);
    infoForm->addRow("描述:", smDesc_);
    infoForm->addRow("PCD 文件:", smMapFile_);
    infoForm->addRow("点云信息:", smCloudInfo_);
    layout->addWidget(infoGroup);

    // Transform
    auto* tfGroup = new QGroupBox("变换 (Transform → default_submap)");
    auto* tfForm = new QFormLayout(tfGroup);
    smTx_ = makeSpinBox(); smTy_ = makeSpinBox(); smTz_ = makeSpinBox();
    smRx_ = makeSpinBox(-1, 1); smRy_ = makeSpinBox(-1, 1);
    smRz_ = makeSpinBox(-1, 1); smRw_ = makeSpinBox(-1, 1);
    tfForm->addRow("tx:", smTx_);
    tfForm->addRow("ty:", smTy_);
    tfForm->addRow("tz:", smTz_);
    tfForm->addRow("qx:", smRx_);
    tfForm->addRow("qy:", smRy_);
    tfForm->addRow("qz:", smRz_);
    tfForm->addRow("qw:", smRw_);
    layout->addWidget(tfGroup);

    // 显示选项
    auto* dispGroup = new QGroupBox("显示选项");
    auto* dispForm = new QFormLayout(dispGroup);
    smColorBtn_ = new QPushButton("选择颜色");
    smVoxelSize_ = makeSpinBox(0.01, 1.0, 3);
    smVoxelSize_->setValue(0.05);
    smPointSize_ = makeSpinBox(1, 20, 1);
    smPointSize_->setValue(2.0);
    dispForm->addRow("点云颜色:", smColorBtn_);
    dispForm->addRow("降采样体素(m):", smVoxelSize_);
    dispForm->addRow("点大小:", smPointSize_);
    layout->addWidget(dispGroup);

    layout->addStretch();

    // 应用按钮
    auto* applyBtn = new QPushButton("应用修改");
    layout->addWidget(applyBtn);

    // ── 连接信号 ─────────────────────────────────────────────────────
    connect(smColorBtn_, &QPushButton::clicked, [this]() {
        if (!currentSm_) return;
        QColor c = QColorDialog::getColor(currentSm_->displayOptions().color, this, "选择点云颜色");
        if (c.isValid()) {
            currentSm_->displayOptions().color = c;
            smColorBtn_->setStyleSheet(QString("background-color: %1").arg(c.name()));
            emit colorChanged(currentSm_->id(), c);
        }
    });

    connect(applyBtn, &QPushButton::clicked, [this]() {
        if (!currentSm_) return;
        // 写回属性
        currentSm_->setName(smName_->text());
        currentSm_->setDescription(smDesc_->text());

        RigidTransform tf;
        tf.translation = Eigen::Vector3d(smTx_->value(), smTy_->value(), smTz_->value());
        tf.rotation = Eigen::Quaterniond(smRw_->value(), smRx_->value(), smRy_->value(), smRz_->value()).normalized();
        currentSm_->setTransform(tf);

        currentSm_->displayOptions().voxelSize = smVoxelSize_->value();
        currentSm_->displayOptions().pointSize = smPointSize_->value();
        currentSm_->updateDisplayCloud(smVoxelSize_->value());

        emit voxelSizeChanged(currentSm_->id(), smVoxelSize_->value());
        emit submapPropertiesChanged(currentSm_->id());
    });

    stack_->addWidget(smPage_);
}

void PropertiesPanel::showSubmapProperties(SubMap* sm) {
    if (!sm) { clear(); return; }
    currentSm_ = sm;

    smId_->setText(sm->id());
    smName_->setText(sm->name());
    smDesc_->setText(sm->description());
    smMapFile_->setText(sm->mapFile());

    if (sm->cloud()) {
        smCloudInfo_->setText(QString("原始点数: %1\n显示点数: %2")
                                  .arg(sm->cloud()->size())
                                  .arg(sm->displayCloud() ? sm->displayCloud()->size() : 0));
    } else {
        smCloudInfo_->setText("未加载");
    }

    const auto& tf = sm->transform();
    smTx_->setValue(tf.translation.x());
    smTy_->setValue(tf.translation.y());
    smTz_->setValue(tf.translation.z());
    smRx_->setValue(tf.rotation.x());
    smRy_->setValue(tf.rotation.y());
    smRz_->setValue(tf.rotation.z());
    smRw_->setValue(tf.rotation.w());

    const auto& disp = sm->displayOptions();
    smColorBtn_->setStyleSheet(QString("background-color: %1").arg(disp.color.name()));
    smVoxelSize_->setValue(disp.voxelSize);
    smPointSize_->setValue(disp.pointSize);

    stack_->setCurrentIndex(1);
}

// ═══════════════════════════════════════════════════════════════════════
//  SemanticPoint 属性页
// ═══════════════════════════════════════════════════════════════════════

void PropertiesPanel::setupSemanticPointPage() {
    spPage_ = new QWidget();
    auto* layout = new QVBoxLayout(spPage_);

    auto* group = new QGroupBox("语义点位属性");
    auto* form = new QFormLayout(group);
    spName_ = new QLineEdit();
    spType_ = new QComboBox();
    spType_->addItems({"single_pose", "multi_pose"});
    spPx_ = makeSpinBox(); spPy_ = makeSpinBox(); spPz_ = makeSpinBox();
    spOx_ = makeSpinBox(-1,1); spOy_ = makeSpinBox(-1,1);
    spOz_ = makeSpinBox(-1,1); spOw_ = makeSpinBox(-1,1);

    form->addRow("名称:", spName_);
    form->addRow("类型:", spType_);
    form->addRow("position.x:", spPx_);
    form->addRow("position.y:", spPy_);
    form->addRow("position.z:", spPz_);
    form->addRow("orient.x:", spOx_);
    form->addRow("orient.y:", spOy_);
    form->addRow("orient.z:", spOz_);
    form->addRow("orient.w:", spOw_);
    layout->addWidget(group);

    layout->addStretch();

    auto* applyBtn = new QPushButton("应用修改");
    layout->addWidget(applyBtn);

    connect(applyBtn, &QPushButton::clicked, [this]() {
        if (!spSm_ || spCurrentName_.isEmpty()) return;

        // 如果名称改变了, 先删旧的
        auto newName = spName_->text();
        if (newName != spCurrentName_) {
            spSm_->removeSemanticPoint(spCurrentName_);
        }

        SemanticPoint sp;
        sp.name = newName;
        sp.type = spType_->currentText();
        Pose pose;
        pose.position = Eigen::Vector3d(spPx_->value(), spPy_->value(), spPz_->value());
        pose.orientation = Eigen::Quaterniond(spOw_->value(), spOx_->value(), spOy_->value(), spOz_->value()).normalized();
        sp.poses.append(pose);
        spSm_->addSemanticPoint(sp);

        spCurrentName_ = newName;
        emit semanticPointChanged(spSm_->id(), newName);
    });

    stack_->addWidget(spPage_);
}

void PropertiesPanel::showSemanticPointProperties(SubMap* sm, const QString& pointName) {
    if (!sm) { clear(); return; }
    auto* sp = sm->semanticPoint(pointName);
    if (!sp) { clear(); return; }

    spSm_ = sm;
    spCurrentName_ = pointName;
    spName_->setText(sp->name);
    spType_->setCurrentText(sp->type);

    if (!sp->poses.isEmpty()) {
        const auto& pose = sp->poses.first();
        spPx_->setValue(pose.position.x());
        spPy_->setValue(pose.position.y());
        spPz_->setValue(pose.position.z());
        spOx_->setValue(pose.orientation.x());
        spOy_->setValue(pose.orientation.y());
        spOz_->setValue(pose.orientation.z());
        spOw_->setValue(pose.orientation.w());
    }

    stack_->setCurrentIndex(2);
}

// ═══════════════════════════════════════════════════════════════════════
//  Portal 属性页
// ═══════════════════════════════════════════════════════════════════════

void PropertiesPanel::setupPortalPage() {
    ptPage_ = new QWidget();
    auto* layout = new QVBoxLayout(ptPage_);

    auto* group = new QGroupBox("传送门属性");
    auto* form = new QFormLayout(group);
    ptName_   = new QLineEdit();
    ptTarget_ = new QComboBox();
    form->addRow("传送门名称:", ptName_);
    form->addRow("目标 Submap:", ptTarget_);
    layout->addWidget(group);

    layout->addStretch();

    auto* applyBtn = new QPushButton("应用修改");
    layout->addWidget(applyBtn);

    connect(applyBtn, &QPushButton::clicked, [this]() {
        if (!ptSm_ || ptIdx_ < 0) return;
        auto portals = ptSm_->portals();
        if (ptIdx_ >= portals.size()) return;
        // 直接修改: 先清除再加
        Portal p;
        p.portalName = ptName_->text();
        p.targetSubmap = ptTarget_->currentText();
        // 简单方式: 重建 portals 列表
        auto allPortals = ptSm_->portals();
        allPortals[ptIdx_] = p;
        ptSm_->clearPortals();
        for (const auto& pt : allPortals) ptSm_->addPortal(pt);

        emit portalChanged(ptSm_->id(), ptIdx_);
    });

    stack_->addWidget(ptPage_);
}

void PropertiesPanel::showPortalProperties(SubMap* sm, int portalIdx, const QStringList& allSubmapIds) {
    if (!sm || portalIdx < 0 || portalIdx >= sm->portals().size()) { clear(); return; }

    ptSm_  = sm;
    ptIdx_ = portalIdx;

    const auto& p = sm->portals()[portalIdx];
    ptName_->setText(p.portalName);
    ptTarget_->clear();
    ptTarget_->addItems(allSubmapIds);
    ptTarget_->setCurrentText(p.targetSubmap);

    stack_->setCurrentIndex(3);
}

void PropertiesPanel::clear() {
    stack_->setCurrentIndex(0);
    currentSm_ = nullptr;
    spSm_ = nullptr;
    ptSm_ = nullptr;
}

} // namespace me
