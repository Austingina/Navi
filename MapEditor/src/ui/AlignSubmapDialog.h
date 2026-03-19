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

namespace me {

class AlignSubmapDialog : public QDialog {
    Q_OBJECT
public:
    explicit AlignSubmapDialog(MapProject* project,
                                const QString& submapId,
                                QWidget* parent = nullptr);

    /// 设置在 default_submap 上选择的初始位置
    void setGuessPosition(const Eigen::Vector3d& pos);

    /// 获取匹配结果
    const MatchResult& matchResult() const { return result_; }

signals:
    /// 请求用户在 viewer 上选择一个点
    void requestPickPoint();
    /// 匹配成功后通知可预览匹配结果（MainWindow 将临时应用该变换以作预览）
    void previewAvailable();

private slots:
    void onStartAlign();

private:
    void setupUI();
    void updateProgress(int percent, const QString& msg);

    MapProject* project_;
    QString     submapId_;
    Eigen::Vector3d guessPos_{0, 0, 0};
    bool        hasGuess_ = false;

    // UI
    QLabel*         lblGuessPos_ = nullptr;
    QDoubleSpinBox* spinSearchXY_ = nullptr;
    QDoubleSpinBox* spinSearchZ_ = nullptr;
    QDoubleSpinBox* spinCropSize_ = nullptr;
    QDoubleSpinBox* spinMaxFitness_ = nullptr;
    QProgressBar*   progressBar_ = nullptr;
    QLabel*         lblStatus_ = nullptr;
    QPushButton*    btnStart_ = nullptr;
    QPushButton*    btnAccept_ = nullptr;

    MatchResult result_;
};

} // namespace me
