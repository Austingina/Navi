#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  BuildVprDatabaseDialog — 调用 vpr_preprocess.py 为某个 submap 构建
//  VPR 特征数据库的对话框（带实时日志窗）
//
//  当前对话框支持两种数据来源（顶部 Tab 切换）：
//   * S20  (ROS1 bag + ImgPose.txt, PolyFisheye 去畸变可选)
//   * R1Pro (ROS2 .db3 bag + /Odometry, 自动 PinHole 去畸变 + 中心裁剪)
//
//  --- BuildVprParams ---
//  为了让用户在两个 tab 之间切换时保留各自的输入，参数结构里同时存了 S20
//  与 R1Pro 两套字段；``source`` 字段标记当前激活的 tab。
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
class QTabWidget;
class QWidget;

namespace me {

/// 一次 VPR 构建的所有输入参数，便于 caller 记忆 / 复用
struct BuildVprParams {
    /// 当前激活的数据来源 tab："ros1" (S20) 或 "ros2_r1pro" (R1Pro)
    QString source           = "ros1";

    // ── S20 / ROS1 路径 ────────────────────────────────────────────
    QString s20BagPath;
    QString poseFile;
    QString s20ImageTopic    = "/camera_agent/img_left/compressed";
    QString calibrationFile;        // 空 == 使用 PolyFisheyeUndistorter 默认 (S20)
    bool    undistortFisheye = true;
    int     outWidth         = 1024;
    int     outHeight        = 768;
    double  fov              = 90.0;

    // ── R1Pro / ROS2 路径 ─────────────────────────────────────────
    /// ROS2 bag 目录（也可以是直接指向 *.db3 的路径）
    QString r1proBagPath;
    QString poseTopic        = "/Odometry";
    QString r1proImageTopic  = "/hdas/camera_head/left_raw/image_raw_color/compressed";
    /// PinHole 去畸变后中心裁剪尺寸；必须与 vpr_bootstrap._VPR_QUERY_TARGET_W/H 一致
    int     cropWidth        = 1024;
    int     cropHeight       = 768;

    // ── 公共参数 ────────────────────────────────────────────────
    /// 数据库采样间距 (m)。与 vpr_preprocess.py 的 --db_distance 默认值
    /// (0.2) 对齐：开启低纹理过滤后，按 0.2m 取样能覆盖室内多楼层场景
    /// 又不至于让数据库过大。
    double  dbDistance       = 0.2;
    QString method           = "salad-indoor";
    QString device           = "cuda";
    int     batchSize        = 4;
    int     numWorkers       = 4;
    /// 提取出的 JPG 缓存目录；空时由 MainWindow 决定默认值
    /// (一般为 ~/.vpr/<projectName>/<submapId>/)
    QString imageCacheDir;

    // ── 低纹理帧过滤 (L1 + L2，见 utils_common.assess_frame_quality) ─
    /// false → 给 vpr_preprocess.py 加 --no_texture_filter，整段跳过
    bool    textureFilter    = true;
    /// L1：Laplacian 方差阈值，低于此判为模糊/无纹理（白墙/吊顶）。
    /// 默认 20.0 偏宽松，避免在弱纹理走廊里把过多关键帧丢掉；如需更激进
    /// 的过滤可在对话框里调到 60+ 再观察 low_texture_filter_report.csv。
    double  minLapVar        = 20.0;
    /// L2：ORB 关键点数下限，少于此判为低判别力（玻璃反光/纯地板）
    int     minOrbKpts       = 80;
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
    void onBrowseS20Bag();
    void onBrowsePose();
    void onBrowseCalibration();
    void onBrowseR1proBag();
    void onBrowseImageCache();
    void onUndistortToggled(bool enabled);
    void onTextureFilterToggled(bool enabled);
    void onRun();
    void onStop();
    void onProcessOutput();
    void onProcessFinished(int exitCode, int exitStatus);

private:
    void buildUI();
    QWidget* buildS20Tab();
    QWidget* buildR1proTab();
    void populateFromParams(const BuildVprParams& p);
    bool validate(QString* err) const;
    QStringList buildArgList(const QString& scriptPath) const;
    void appendLog(const QString& text);

    /// 当前激活 tab 对应的 source 字符串 (与 vpr_preprocess.py --source 取值一致)
    QString activeSource() const;

    static QString findVprPython();
    static QString findVprScript();
    static QString stripAnsi(const QString& s);

    QString submapId_;
    QString outputDir_;

    QTabWidget* tabs_ = nullptr;

    // ── S20 / ROS1 tab ─────────────────────────────────────────────
    QLineEdit*       editS20Bag_      = nullptr;
    QLineEdit*       editPose_        = nullptr;
    QLineEdit*       editS20Topic_    = nullptr;
    QLineEdit*       editCalib_       = nullptr;
    QCheckBox*       chkUndistort_    = nullptr;
    QSpinBox*        spinOutW_        = nullptr;
    QSpinBox*        spinOutH_        = nullptr;
    QDoubleSpinBox*  spinFov_         = nullptr;

    // ── R1Pro / ROS2 tab ───────────────────────────────────────────
    QLineEdit*       editR1proBag_    = nullptr;
    QLineEdit*       editPoseTopic_   = nullptr;
    QLineEdit*       editR1proTopic_  = nullptr;
    QSpinBox*        spinCropW_       = nullptr;
    QSpinBox*        spinCropH_       = nullptr;

    // ── 公共参数 ────────────────────────────────────────────────
    QDoubleSpinBox*  spinDbDist_      = nullptr;
    QComboBox*       cmbMethod_       = nullptr;
    QComboBox*       cmbDevice_       = nullptr;
    QSpinBox*        spinBatch_       = nullptr;
    QSpinBox*        spinWorkers_     = nullptr;
    QLineEdit*       editOutputDir_       = nullptr;
    QLineEdit*       editImageCacheDir_   = nullptr;
    QString          defaultImageCacheDir_;

    // ── 低纹理帧过滤 (L1 + L2) ──────────────────────────────────
    QCheckBox*       chkTextureFilter_ = nullptr;
    QDoubleSpinBox*  spinMinLapVar_    = nullptr;
    QSpinBox*        spinMinOrbKpts_   = nullptr;

    QPlainTextEdit*  logView_         = nullptr;
    QPushButton*     btnRun_          = nullptr;
    QPushButton*     btnStop_         = nullptr;
    QPushButton*     btnClose_        = nullptr;

    QPointer<QProcess> process_;
    bool finishedOk_ = false;
};

} // namespace me
