#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  BuildVprDatabaseDialog — 调用 vpr_preprocess.py 为某个 submap 构建
//  VPR 特征数据库的对话框（带实时日志窗）
// ═══════════════════════════════════════════════════════════════════════

#include <QDialog>
#include <QPointer>
#include <QString>
#include <QStringList>

class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPlainTextEdit;
class QProcess;
class QPushButton;
class QSpinBox;

namespace me {

/// 一次 VPR 构建的所有输入参数，便于 caller 记忆 / 复用
struct BuildVprParams {
    QString bagPath;
    QString poseFile;
    QString imageTopic       = "/camera_agent/img_left/compressed";
    QString calibrationFile;        // 空 == 使用 PolyFisheyeUndistorter 默认 (S20)
    bool    undistortFisheye = true;
    double  dbDistance       = 1.0;
    int     outWidth         = 1024;
    int     outHeight        = 768;
    double  fov              = 90.0;
    QString method           = "salad-indoor";
    QString device           = "cuda";
    int     batchSize        = 4;
    int     numWorkers       = 4;
    /// 提取出的 JPG 缓存目录；空时由 MainWindow 决定默认值
    /// (一般为 ~/.vpr/<projectName>/<submapId>/)
    QString imageCacheDir;
};

class BuildVprDatabaseDialog : public QDialog {
    Q_OBJECT
public:
    BuildVprDatabaseDialog(const QString& submapId,
                           const QString& outputDir,
                           const QString& defaultImageCacheDir,
                           const BuildVprParams& initial,
                           QWidget* parent = nullptr);
    ~BuildVprDatabaseDialog() override;

    /// 用户当前对话框里的参数（不论是否运行过），用于上层缓存
    BuildVprParams currentParams() const;

    /// 上一次运行是否以 exit code 0 正常结束
    bool ranSuccessfully() const { return finishedOk_; }

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void onBrowseBag();
    void onBrowsePose();
    void onBrowseCalibration();
    void onBrowseImageCache();
    void onUndistortToggled(bool enabled);
    void onRun();
    void onStop();
    void onProcessOutput();
    void onProcessFinished(int exitCode, int exitStatus);

private:
    void buildUI();
    void populateFromParams(const BuildVprParams& p);
    bool validate(QString* err) const;
    QStringList buildArgList(const QString& scriptPath) const;
    void appendLog(const QString& text);

    static QString findVprPython();
    static QString findVprScript();
    static QString stripAnsi(const QString& s);

    QString submapId_;
    QString outputDir_;

    QLineEdit*       editBag_       = nullptr;
    QLineEdit*       editPose_      = nullptr;
    QLineEdit*       editTopic_     = nullptr;
    QLineEdit*       editCalib_     = nullptr;
    QCheckBox*       chkUndistort_  = nullptr;
    QDoubleSpinBox*  spinDbDist_    = nullptr;
    QSpinBox*        spinOutW_      = nullptr;
    QSpinBox*        spinOutH_      = nullptr;
    QDoubleSpinBox*  spinFov_       = nullptr;
    QComboBox*       cmbMethod_     = nullptr;
    QComboBox*       cmbDevice_     = nullptr;
    QSpinBox*        spinBatch_     = nullptr;
    QSpinBox*        spinWorkers_   = nullptr;
    QLineEdit*       editOutputDir_     = nullptr;
    QLineEdit*       editImageCacheDir_ = nullptr;
    QString          defaultImageCacheDir_;
    QPlainTextEdit*  logView_           = nullptr;
    QPushButton*     btnRun_        = nullptr;
    QPushButton*     btnStop_       = nullptr;
    QPushButton*     btnClose_      = nullptr;

    QPointer<QProcess> process_;
    bool finishedOk_ = false;
};

} // namespace me
