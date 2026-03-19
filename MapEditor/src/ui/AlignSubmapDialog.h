#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  AlignSubmapDialog — 子地图对齐对话框 (3D-BBS + GICP)
// ═══════════════════════════════════════════════════════════════════════

#include "core/MapProject.h"
#include "algorithms/CloudMatcher.h"
#include <QDialog>
#include <QProgressBar>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>

namespace me {

class AlignSubmapDialog : public QDialog {
    Q_OBJECT
public:
    explicit AlignSubmapDialog(MapProject* project,
                                const QString& submapId,
                                QWidget* parent = nullptr);

    void setGuessPosition(const Eigen::Vector3d& pos);
    void setSrcCropCenter(const Eigen::Vector3d& localPos);

    const MatchResult& matchResult() const { return result_; }
    const QString& submapId() const { return submapId_; }

    Eigen::Vector3d guessPosition() const { return guessPos_; }
    Eigen::Vector3d srcCropCenter() const { return srcCropCenter_; }
    double searchRangeXY() const { return spinSearchXY_->value(); }
    double searchRangeZ() const { return spinSearchZ_->value(); }
    double cropSize() const { return spinCropSize_->value(); }

signals:
    void requestPickPoint();
    void requestPickSourceCenter();
    void requestPreview();
    void previewAvailable();

private slots:
    void onStartAlign();

private:
    void setupUI();
    void updateProgress(int percent, const QString& msg);

    MapProject* project_;
    QString     submapId_;
    Eigen::Vector3d guessPos_{0, 0, 0};
    Eigen::Vector3d srcCropCenter_{0, 0, 0};
    bool        hasGuess_ = false;

    // UI — 初始位置
    QLabel*         lblGuessPos_ = nullptr;
    // UI — 源点云裁剪中心
    QLabel*         lblSrcCenter_ = nullptr;

    // UI — 目标点云组
    QDoubleSpinBox* spinSearchXY_ = nullptr;
    QDoubleSpinBox* spinSearchZ_ = nullptr;

    // UI — 源点云组
    QDoubleSpinBox* spinCropSize_ = nullptr;
    QDoubleSpinBox* spinBbsDownsample_ = nullptr;

    // UI — 3D-BBS 组
    QDoubleSpinBox* spinBbsMinLevelRes_ = nullptr;
    QSpinBox*       spinBbsMaxLevel_ = nullptr;
    QDoubleSpinBox* spinBbsScoreThresh_ = nullptr;

    // UI — GICP 组
    QDoubleSpinBox* spinMaxFitness_ = nullptr;

    // UI — 其他
    QProgressBar*   progressBar_ = nullptr;
    QLabel*         lblStatus_ = nullptr;
    QPushButton*    btnStart_ = nullptr;
    QPushButton*    btnAccept_ = nullptr;

    MatchResult result_;
};

} // namespace me
