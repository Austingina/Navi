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
        "1. 在 default_submap 上点击一个假想位置 (新 submap 原点的大概位置)\n"
        "2. 从新 submap 原点附近裁剪匹配点云\n"
        "3. 3D-BBS 在假想位置附近搜索最佳匹配\n"
        "4. GICP 精配准验证 (fitness ≤ 阈值则确认)");
    desc->setWordWrap(true);
    layout->addWidget(desc);

    // ── 假想位置 ─────────────────────────────────────────────────────
    auto* guessGroup = new QGroupBox("假想位置");
    auto* guessLayout = new QHBoxLayout(guessGroup);
    guessLayout->setContentsMargins(10, 20, 10, 10);
    lblGuessPos_ = new QLabel("未选择");
    auto* btnPick = new QPushButton("在地图上选择点");
    guessLayout->addWidget(lblGuessPos_);
    guessLayout->addWidget(btnPick);
    layout->addWidget(guessGroup);

    // 默认假想位置为 (0,0,0) —— 表示新 submap 原点与 default_submap 重合
    setGuessPosition(Eigen::Vector3d(0.0, 0.0, 0.0));

    connect(btnPick, &QPushButton::clicked, [this]() {
        emit requestPickPoint();
    });

    // ── 搜索参数 ─────────────────────────────────────────────────────
    auto* paramGroup = new QGroupBox("搜索参数");
    auto* paramForm = new QFormLayout(paramGroup);
    paramForm->setContentsMargins(10, 20, 10, 10);

    spinSearchXY_ = new QDoubleSpinBox();
    spinSearchXY_->setRange(1, 200); spinSearchXY_->setValue(100.0); spinSearchXY_->setSuffix(" m");
    paramForm->addRow("XY 搜索范围:", spinSearchXY_);

    spinSearchZ_ = new QDoubleSpinBox();
    spinSearchZ_->setRange(0.5, 50); spinSearchZ_->setValue(20.0); spinSearchZ_->setSuffix(" m");
    paramForm->addRow("Z 搜索范围:", spinSearchZ_);

    spinCropSize_ = new QDoubleSpinBox();
    spinCropSize_->setRange(5, 50); spinCropSize_->setValue(20.0); spinCropSize_->setSuffix(" m");
    paramForm->addRow("裁剪范围 (立方):", spinCropSize_);

    spinMaxFitness_ = new QDoubleSpinBox();
    spinMaxFitness_->setRange(0.01, 2.0); spinMaxFitness_->setDecimals(3);
    spinMaxFitness_->setValue(1.2); spinMaxFitness_->setSingleStep(0.05);
    paramForm->addRow("最大 fitness 阈值:", spinMaxFitness_);

    layout->addWidget(paramGroup);

    // ── 进度 ─────────────────────────────────────────────────────────
    progressBar_ = new QProgressBar();
    progressBar_->setRange(0, 100);
    progressBar_->setValue(0);
    layout->addWidget(progressBar_);

    lblStatus_ = new QLabel("就绪");
    layout->addWidget(lblStatus_);

    // ── 按钮 ─────────────────────────────────────────────────────────
    auto* btnLayout = new QHBoxLayout();
    btnStart_  = new QPushButton("开始对齐");
    btnAccept_ = new QPushButton("应用结果");
    btnAccept_->setEnabled(false);
    auto* btnCancel = new QPushButton("取消");

    btnLayout->addStretch();
    btnLayout->addWidget(btnStart_);
    btnLayout->addWidget(btnAccept_);
    btnLayout->addWidget(btnCancel);
    layout->addLayout(btnLayout);

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

void AlignSubmapDialog::updateProgress(int percent, const QString& msg) {
    progressBar_->setValue(percent);
    lblStatus_->setText(msg);
    QApplication::processEvents();
}

void AlignSubmapDialog::onStartAlign() {
    if (!hasGuess_) {
        QMessageBox::warning(this, "提示", "请先在地图上选择假想位置");
        return;
    }

    auto* sm = project_->submap(submapId_);
    auto* defSm = project_->submap(project_->defaultSubmapId());
    if (!sm || !defSm) {
        QMessageBox::warning(this, "错误", "找不到 submap 数据");
        return;
    }
    if (!sm->cloud() || !defSm->cloud()) {
        QMessageBox::warning(this, "错误", "子地图缺少点云数据");
        return;
    }

    btnStart_->setEnabled(false);
    btnAccept_->setEnabled(false);

    // 构造参数
    MatchParams params;
    params.searchRangeXY = spinSearchXY_->value();
    params.searchRangeZ  = spinSearchZ_->value();
    params.cropSize      = spinCropSize_->value();
    params.maxFitness    = spinMaxFitness_->value();

    // 执行匹配
    CloudMatcher matcher;
    matcher.setTarget(defSm->cloud());
    matcher.setSource(sm->cloud());

    result_ = matcher.match(guessPos_, params,
        [this](int p, const QString& msg) { updateProgress(p, msg); });

    if (result_.success) {
        lblStatus_->setText(QString("✅ 匹配成功! fitness=%1, RMSE=%2")
                                .arg(result_.fitness, 0, 'f', 4)
                                .arg(result_.rmse, 0, 'f', 4));
        btnAccept_->setEnabled(true);
        // 通知主窗口可以预览匹配结果
        emit previewAvailable();
    } else {
        lblStatus_->setText("❌ " + result_.message);
    }

    btnStart_->setEnabled(true);
}

} // namespace me
