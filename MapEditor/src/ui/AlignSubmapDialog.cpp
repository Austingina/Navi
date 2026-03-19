#include "ui/AlignSubmapDialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QMessageBox>
#include <QApplication>
#include <QThread>

namespace me {

AlignSubmapDialog::AlignSubmapDialog(MapProject* project,
                                       const QString& submapId,
                                       QWidget* parent)
    : QDialog(parent), project_(project), submapId_(submapId) {
    setWindowTitle(QString("对齐子地图: %1").arg(submapId));
    setMinimumWidth(500);
    setupUI();
}

void AlignSubmapDialog::setupUI() {
    auto* layout = new QVBoxLayout(this);

    // ── 说明 ─────────────────────────────────────────────────────────
    auto* desc = new QLabel(
        "对齐流程:\n"
        "1. 在目标子地图上点击一个初始位置 (新 submap 的大概位置)\n"
        "2. 可选: 在源子地图上选择裁剪中心 (默认原点)\n"
        "3. 点击\"预览裁剪框\"查看裁剪区域\n"
        "4. 3D-BBS 在初始位置附近搜索最佳匹配\n"
        "5. GICP 精配准验证 (fitness ≤ 阈值则确认)");
    desc->setWordWrap(true);
    layout->addWidget(desc);

    // ── 初始位置 ─────────────────────────────────────────────────────
    auto* guessGroup = new QGroupBox("初始位置 (目标子地图)");
    auto* guessLayout = new QHBoxLayout(guessGroup);
    guessLayout->setContentsMargins(10, 20, 10, 10);
    lblGuessPos_ = new QLabel("未选择");
    auto* btnPick = new QPushButton("在地图上选择点");
    guessLayout->addWidget(lblGuessPos_);
    guessLayout->addWidget(btnPick);
    layout->addWidget(guessGroup);

    setGuessPosition(Eigen::Vector3d(0.0, 0.0, 0.0));

    connect(btnPick, &QPushButton::clicked, [this]() {
        emit requestPickPoint();
    });

    // ── 目标点云参数 ─────────────────────────────────────────────────
    auto* tgtGroup = new QGroupBox("目标点云");
    auto* tgtForm = new QFormLayout(tgtGroup);
    tgtForm->setContentsMargins(10, 20, 10, 10);

    spinSearchXY_ = new QDoubleSpinBox();
    spinSearchXY_->setRange(1, 200); spinSearchXY_->setValue(100.0); spinSearchXY_->setSuffix(" m");
    tgtForm->addRow("XY 搜索范围:", spinSearchXY_);

    spinSearchZ_ = new QDoubleSpinBox();
    spinSearchZ_->setRange(0.5, 50); spinSearchZ_->setValue(20.0); spinSearchZ_->setSuffix(" m");
    tgtForm->addRow("Z 搜索范围:", spinSearchZ_);

    layout->addWidget(tgtGroup);

    // ── 源点云参数 ───────────────────────────────────────────────────
    auto* srcGroup = new QGroupBox("源点云");
    auto* srcForm = new QFormLayout(srcGroup);
    srcForm->setContentsMargins(10, 20, 10, 10);

    // 裁剪中心选择
    auto* centerLayout = new QHBoxLayout();
    lblSrcCenter_ = new QLabel("(0.00, 0.00, 0.00)");
    auto* btnPickSrcCenter = new QPushButton("在地图上选择裁剪中心");
    centerLayout->addWidget(lblSrcCenter_);
    centerLayout->addWidget(btnPickSrcCenter);
    srcForm->addRow("裁剪中心:", centerLayout);

    connect(btnPickSrcCenter, &QPushButton::clicked, [this]() {
        emit requestPickSourceCenter();
    });

    spinCropSize_ = new QDoubleSpinBox();
    spinCropSize_->setRange(5, 200); spinCropSize_->setValue(20.0); spinCropSize_->setSuffix(" m");
    srcForm->addRow("裁剪范围 (立方):", spinCropSize_);

    spinBbsDownsample_ = new QDoubleSpinBox();
    spinBbsDownsample_->setRange(0.05, 5.0); spinBbsDownsample_->setValue(0.5);
    spinBbsDownsample_->setDecimals(2); spinBbsDownsample_->setSingleStep(0.1);
    spinBbsDownsample_->setSuffix(" m");
    srcForm->addRow("BBS 降采样:", spinBbsDownsample_);

    layout->addWidget(srcGroup);

    // ── 3D-BBS 参数 ─────────────────────────────────────────────────
    auto* bbsGroup = new QGroupBox("3D-BBS");
    auto* bbsForm = new QFormLayout(bbsGroup);
    bbsForm->setContentsMargins(10, 20, 10, 10);

    spinBbsMinLevelRes_ = new QDoubleSpinBox();
    spinBbsMinLevelRes_->setRange(0.1, 10.0); spinBbsMinLevelRes_->setValue(1.0);
    spinBbsMinLevelRes_->setDecimals(2); spinBbsMinLevelRes_->setSingleStep(0.1);
    spinBbsMinLevelRes_->setSuffix(" m");
    bbsForm->addRow("最细层分辨率:", spinBbsMinLevelRes_);

    spinBbsMaxLevel_ = new QSpinBox();
    spinBbsMaxLevel_->setRange(1, 10); spinBbsMaxLevel_->setValue(6);
    bbsForm->addRow("体素层级数:", spinBbsMaxLevel_);

    spinBbsScoreThresh_ = new QDoubleSpinBox();
    spinBbsScoreThresh_->setRange(0.01, 1.0); spinBbsScoreThresh_->setValue(0.1);
    spinBbsScoreThresh_->setDecimals(3); spinBbsScoreThresh_->setSingleStep(0.01);
    bbsForm->addRow("最低匹配得分:", spinBbsScoreThresh_);

    layout->addWidget(bbsGroup);

    // ── GICP 参数 ───────────────────────────────────────────────────
    auto* gicpGroup = new QGroupBox("GICP");
    auto* gicpForm = new QFormLayout(gicpGroup);
    gicpForm->setContentsMargins(10, 20, 10, 10);

    spinMaxFitness_ = new QDoubleSpinBox();
    spinMaxFitness_->setRange(0.01, 2.0); spinMaxFitness_->setDecimals(3);
    spinMaxFitness_->setValue(1.2); spinMaxFitness_->setSingleStep(0.05);
    gicpForm->addRow("最大 fitness 阈值:", spinMaxFitness_);

    layout->addWidget(gicpGroup);

    // ── 进度 ─────────────────────────────────────────────────────────
    progressBar_ = new QProgressBar();
    progressBar_->setRange(0, 100);
    progressBar_->setValue(0);
    layout->addWidget(progressBar_);

    lblStatus_ = new QLabel("就绪");
    layout->addWidget(lblStatus_);

    // ── 按钮 ─────────────────────────────────────────────────────────
    auto* btnLayout = new QHBoxLayout();
    auto* btnPreview = new QPushButton("预览裁剪框");
    btnStart_  = new QPushButton("开始对齐");
    btnAccept_ = new QPushButton("应用结果");
    btnAccept_->setEnabled(false);
    auto* btnCancel = new QPushButton("取消");

    btnLayout->addStretch();
    btnLayout->addWidget(btnPreview);
    btnLayout->addWidget(btnStart_);
    btnLayout->addWidget(btnAccept_);
    btnLayout->addWidget(btnCancel);
    layout->addLayout(btnLayout);

    connect(btnPreview, &QPushButton::clicked, [this]() {
        emit requestPreview();
    });
    connect(btnStart_,  &QPushButton::clicked, this, &AlignSubmapDialog::onStartAlign);
    connect(btnAccept_, &QPushButton::clicked, this, &QDialog::accept);
    connect(btnCancel,  &QPushButton::clicked, this, &QDialog::reject);
}

void AlignSubmapDialog::setGuessPosition(const Eigen::Vector3d& pos) {
    guessPos_ = pos;
    hasGuess_ = true;
    lblGuessPos_->setText(QString("(%1, %2, %3)")
                              .arg(pos.x(), 0, 'f', 2)
                              .arg(pos.y(), 0, 'f', 2)
                              .arg(pos.z(), 0, 'f', 2));
}

void AlignSubmapDialog::setSrcCropCenter(const Eigen::Vector3d& localPos) {
    srcCropCenter_ = localPos;
    lblSrcCenter_->setText(QString("(%1, %2, %3)")
                               .arg(localPos.x(), 0, 'f', 2)
                               .arg(localPos.y(), 0, 'f', 2)
                               .arg(localPos.z(), 0, 'f', 2));
}

void AlignSubmapDialog::updateProgress(int percent, const QString& msg) {
    progressBar_->setValue(percent);
    lblStatus_->setText(msg);
    QApplication::processEvents();
}

void AlignSubmapDialog::onStartAlign() {
    if (!hasGuess_) {
        QMessageBox::warning(this, "提示", "请先在地图上选择初始位置");
        return;
    }

    auto* sm = project_->submap(submapId_);
    if (!sm) {
        QMessageBox::warning(this, "错误",
            QString("找不到子地图: %1").arg(submapId_));
        return;
    }

    QString targetId = project_->defaultSubmapId();
    if (targetId.isEmpty() || !project_->submap(targetId)) {
        for (const auto& id : project_->submapIds()) {
            if (id != submapId_) { targetId = id; break; }
        }
    }
    auto* defSm = project_->submap(targetId);
    if (!defSm) {
        QMessageBox::warning(this, "错误", "需要至少两个子地图才能进行对齐");
        return;
    }

    if (!sm->cloud() || !defSm->cloud()) {
        QMessageBox::warning(this, "错误", "子地图缺少点云数据");
        return;
    }

    btnStart_->setEnabled(false);
    btnAccept_->setEnabled(false);

    MatchParams params;
    params.searchRangeXY   = spinSearchXY_->value();
    params.searchRangeZ    = spinSearchZ_->value();
    params.cropSize        = spinCropSize_->value();
    params.srcCropCenter   = srcCropCenter_;
    params.bbsDownsample   = static_cast<float>(spinBbsDownsample_->value());
    params.bbsMinLevelRes  = spinBbsMinLevelRes_->value();
    params.bbsMaxLevel     = spinBbsMaxLevel_->value();
    params.bbsScoreThresh  = spinBbsScoreThresh_->value();
    params.maxFitness      = spinMaxFitness_->value();

    CloudMatcher matcher;
    matcher.setTarget(defSm->cloud());
    matcher.setSource(sm->cloud());

    result_ = matcher.match(guessPos_, params,
        [this](int p, const QString& msg) { updateProgress(p, msg); });

    if (result_.success) {
        lblStatus_->setText(QString("匹配成功! fitness=%1, RMSE=%2")
                                .arg(result_.fitness, 0, 'f', 4)
                                .arg(result_.rmse, 0, 'f', 4));
        btnAccept_->setEnabled(true);
        emit previewAvailable();
    } else {
        lblStatus_->setText("❌ " + result_.message);
    }

    btnStart_->setEnabled(true);
}

} // namespace me
