#include "ui/PropertiesPanel.h"

#include <QVBoxLayout>
#include <QScrollArea>
#include <QMessageBox>
#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace me {

namespace {

constexpr double kPi = 3.14159265358979323846;

inline double wrapYawDeg(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

/// Eigen eulerAngles(2,1,0)：先绕 Z、再 Y、再 X；返回的 [0] 为绕 Z 角（yaw）
inline double quaternionYawRadZyx(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond qq = q.normalized();
    Eigen::Vector3d    euler = qq.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler[0];
}

inline Eigen::Quaterniond quaternionWithYawZyxPreservingPitchRollRad(
    double yawRad, const Eigen::Quaterniond& qCurrent) {
    Eigen::Quaterniond qc = qCurrent.normalized();
    Eigen::Vector3d    euler = qc.toRotationMatrix().eulerAngles(2, 1, 0);
    Eigen::Matrix3d R =
        Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
    return Eigen::Quaterniond(R).normalized();
}

} // namespace

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
    infoForm->setContentsMargins(10, 20, 10, 10);
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
    auto* tfGroup = new QGroupBox("变换 (Transform → 父地图)");
    tfGroup_ = tfGroup;
    auto* tfForm = new QFormLayout(tfGroup);
    tfForm->setContentsMargins(10, 20, 10, 10);
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
    dispForm->setContentsMargins(10, 20, 10, 10);
    smColorBtn_ = new QPushButton("选择颜色");
    smVoxelSize_ = makeSpinBox(0.01, 5.0, 3);
    smVoxelSize_->setValue(1.0);
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

    if (tfGroup_) {
        QString parent = sm->parentSubmapId();
        if (parent.isEmpty())
            tfGroup_->setTitle("变换 (Transform — 根节点)");
        else
            tfGroup_->setTitle(QString("变换 (Transform → %1)").arg(parent));
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
    form->setContentsMargins(10, 20, 10, 10);
    spName_ = new QLineEdit();
    spType_ = new QComboBox();
    spType_->addItems({"single_pose", "multi_pose", "LowPrecision_pose", "elevator_call", "elevator_wait", "elevator_in", "elevator_out"});
    spPx_ = makeSpinBox(); spPy_ = makeSpinBox(); spPz_ = makeSpinBox();
    spYaw_ = makeSpinBox(-180.0, 180.0, 2);
    spYaw_->setSuffix(" °");
    spYaw_->setToolTip("绕世界 Z 轴的 yaw（Z-Y-X 顺序中的 Z 角）。修改 yaw 会保留当前 pitch/roll 并更新四元数。");
    spOx_ = makeSpinBox(-1,1); spOy_ = makeSpinBox(-1,1);
    spOz_ = makeSpinBox(-1,1); spOw_ = makeSpinBox(-1,1);

    form->addRow("名称:", spName_);
    form->addRow("类型:", spType_);
    form->addRow("position.x:", spPx_);
    form->addRow("position.y:", spPy_);
    form->addRow("position.z:", spPz_);
    form->addRow("yaw:", spYaw_);
    form->addRow("orient.x:", spOx_);
    form->addRow("orient.y:", spOy_);
    form->addRow("orient.z:", spOz_);
    form->addRow("orient.w:", spOw_);
    layout->addWidget(group);

    spPickPosBtn_ = new QPushButton("从点云选点（填写 position，子地图局部坐标）");
    layout->addWidget(spPickPosBtn_);
    connect(spPickPosBtn_, &QPushButton::clicked, [this]() {
        if (!spSm_) return;
        emit pickSemanticPositionRequested(spSm_->id());
    });

    layout->addStretch();

    auto* applyBtn = new QPushButton("应用修改");
    layout->addWidget(applyBtn);

    auto wireQuatYawSync = [this]() {
        connect(spOx_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &PropertiesPanel::syncSemanticYawFromQuaternionSpins);
        connect(spOy_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &PropertiesPanel::syncSemanticYawFromQuaternionSpins);
        connect(spOz_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &PropertiesPanel::syncSemanticYawFromQuaternionSpins);
        connect(spOw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &PropertiesPanel::syncSemanticYawFromQuaternionSpins);
        connect(spYaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &PropertiesPanel::syncSemanticQuaternionSpinsFromYaw);
    };
    wireQuatYawSync();

    connect(applyBtn, &QPushButton::clicked, [this]() {
        if (!spSm_) return;
        QString newName = spName_->text().trimmed();
        if (newName.isEmpty()) {
            QMessageBox::warning(this, "语义点位", "请填写点位名称。");
            return;
        }

        if (spCurrentName_.isEmpty()) {
            if (spSm_->hasSemanticPoint(newName)) {
                QMessageBox::warning(this, "语义点位",
                    QString("名称 \"%1\" 已存在。").arg(newName));
                return;
            }
        } else {
            if (newName != spCurrentName_ && spSm_->hasSemanticPoint(newName)) {
                QMessageBox::warning(this, "语义点位",
                    QString("名称 \"%1\" 已存在。").arg(newName));
                return;
            }
            if (newName != spCurrentName_) {
                spSm_->removeSemanticPoint(spCurrentName_);
            }
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

void PropertiesPanel::syncSemanticYawFromQuaternionSpins() {
    if (!spYaw_ || !spOx_ || spOrientationSyncGuard_) return;
    spOrientationSyncGuard_ = true;
    Eigen::Quaterniond q(spOw_->value(), spOx_->value(), spOy_->value(), spOz_->value());
    if (q.squaredNorm() < 1e-20) q = Eigen::Quaterniond::Identity();
    else q.normalize();
    const double yawDeg = wrapYawDeg(quaternionYawRadZyx(q) * 180.0 / kPi);
    spYaw_->blockSignals(true);
    spYaw_->setValue(yawDeg);
    spYaw_->blockSignals(false);
    spOrientationSyncGuard_ = false;
}

void PropertiesPanel::syncSemanticQuaternionSpinsFromYaw() {
    if (!spYaw_ || !spOx_ || spOrientationSyncGuard_) return;
    spOrientationSyncGuard_ = true;
    Eigen::Quaterniond q(spOw_->value(), spOx_->value(), spOy_->value(), spOz_->value());
    if (q.squaredNorm() < 1e-20) q = Eigen::Quaterniond::Identity();
    else q.normalize();
    const double yawRad = spYaw_->value() * kPi / 180.0;
    Eigen::Quaterniond qn = quaternionWithYawZyxPreservingPitchRollRad(yawRad, q);
    spOx_->blockSignals(true);
    spOy_->blockSignals(true);
    spOz_->blockSignals(true);
    spOw_->blockSignals(true);
    spOx_->setValue(qn.x());
    spOy_->setValue(qn.y());
    spOz_->setValue(qn.z());
    spOw_->setValue(qn.w());
    spOw_->blockSignals(false);
    spOz_->blockSignals(false);
    spOy_->blockSignals(false);
    spOx_->blockSignals(false);
    spOrientationSyncGuard_ = false;
}

void PropertiesPanel::beginNewSemanticPoint(SubMap* sm) {
    if (!sm) { clear(); return; }
    spSm_ = sm;
    spCurrentName_.clear();
    spName_->clear();
    spType_->setCurrentText("single_pose");
    spPx_->setValue(0.0);
    spPy_->setValue(0.0);
    spPz_->setValue(0.0);
    spOx_->setValue(0.0);
    spOy_->setValue(0.0);
    spOz_->setValue(0.0);
    spOw_->setValue(1.0);
    syncSemanticYawFromQuaternionSpins();
    stack_->setCurrentIndex(2);
}

void PropertiesPanel::setSemanticPositionLocal(double x, double y, double z) {
    if (!spPx_ || !spPy_ || !spPz_) return;
    spPx_->setValue(x);
    spPy_->setValue(y);
    spPz_->setValue(z);
}

void PropertiesPanel::showSemanticPointProperties(SubMap* sm, const QString& pointName) {
    if (!sm) { clear(); return; }
    auto* sp = sm->semanticPoint(pointName);
    if (!sp) { clear(); return; }

    spSm_ = sm;
    spCurrentName_ = pointName;
    spName_->setText(sp->name);
    if (spType_->findText(sp->type) < 0)
        spType_->addItem(sp->type);
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
    } else {
        spPx_->setValue(0.0);
        spPy_->setValue(0.0);
        spPz_->setValue(0.0);
        spOx_->setValue(0.0);
        spOy_->setValue(0.0);
        spOz_->setValue(0.0);
        spOw_->setValue(1.0);
    }
    syncSemanticYawFromQuaternionSpins();

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
    form->setContentsMargins(10, 20, 10, 10);
    ptName_   = new QComboBox();
    ptName_->setMinimumContentsLength(12);
    ptTarget_ = new QComboBox();
    form->addRow("传送门名称 (所在子地图语义点):", ptName_);
    form->addRow("目标 Submap:", ptTarget_);
    layout->addWidget(group);

    layout->addStretch();

    auto* applyBtn = new QPushButton("应用修改");
    layout->addWidget(applyBtn);

    connect(applyBtn, &QPushButton::clicked, [this]() {
        if (!ptSm_ || ptIdx_ < 0) return;

        QString portalName = ptName_->currentText();
        QString targetSm   = ptTarget_->currentText();

        if (portalName.isEmpty()) {
            QMessageBox::warning(this, "传送门", "请选择传送门对应的语义点名称。");
            return;
        }
        if (!ptSm_->hasSemanticPoint(portalName)) {
            QMessageBox::warning(this, "传送门",
                QString("语义点 \"%1\" 不存在，请先在子地图中添加该语义点。").arg(portalName));
            return;
        }
        if (targetSm.isEmpty()) {
            QMessageBox::warning(this, "传送门", "请选择目标 Submap。");
            return;
        }

        auto portals = ptSm_->portals();
        if (ptIdx_ >= portals.size()) return;

        Portal p;
        p.portalName   = portalName;
        p.targetSubmap = targetSm;
        auto allPortals = ptSm_->portals();
        const QString oldName = allPortals[ptIdx_].portalName;
        if (portalName != oldName) {
            for (int i = 0; i < allPortals.size(); ++i) {
                if (i != ptIdx_ && allPortals[i].portalName == portalName) {
                    QMessageBox::warning(this, "传送门",
                        QString("已存在使用该语义点的传送门 \"%1\"。").arg(portalName));
                    return;
                }
            }
        }
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

    QStringList semNames;
    semNames.reserve(sm->semanticPoints().size());
    for (auto it = sm->semanticPoints().constBegin(); it != sm->semanticPoints().constEnd(); ++it) {
        semNames.append(it.key());
    }
    std::sort(semNames.begin(), semNames.end());

    ptName_->clear();
    ptName_->addItems(semNames);
    if (!p.portalName.isEmpty() && ptName_->findText(p.portalName) < 0) {
        ptName_->addItem(p.portalName);
    }
    ptName_->setCurrentText(p.portalName);

    ptTarget_->clear();
    QStringList targets;
    for (const QString& id : allSubmapIds) {
        if (id != sm->id()) {
            targets.append(id);
        }
    }
    std::sort(targets.begin(), targets.end());
    ptTarget_->addItems(targets);
    if (!p.targetSubmap.isEmpty() && ptTarget_->findText(p.targetSubmap) < 0) {
        ptTarget_->addItem(p.targetSubmap);
    }
    ptTarget_->setCurrentText(p.targetSubmap);

    stack_->setCurrentIndex(3);
}

void PropertiesPanel::clear() {
    stack_->setCurrentIndex(0);
    currentSm_ = nullptr;
    spSm_ = nullptr;
    spCurrentName_.clear();
    ptSm_ = nullptr;
    ptIdx_ = -1;
}

} // namespace me
