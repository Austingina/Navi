#include "ui/BuildVprDatabaseDialog.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QRegularExpression>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextCursor>
#include <QVBoxLayout>
#include <QWidget>

namespace me {

namespace {

constexpr int kTabIdxS20   = 0;
constexpr int kTabIdxR1Pro = 1;

constexpr const char* kSourceRos1     = "ros1";
constexpr const char* kSourceRos2R1Pro = "ros2_r1pro";

// 共用度量值，集中在这里方便整体调优
constexpr int kLineEditMinWidth = 360;   // 输入框最小宽度，避免在窄列里被压扁
constexpr int kBrowseBtnWidth   = 96;    // "浏览..." 按钮固定宽度
constexpr int kActionBtnMinW    = 96;    // "运行/停止/关闭" 按钮最小宽度
constexpr int kFormHSpacing     = 12;
constexpr int kFormVSpacing     = 10;

// 统一配置 QFormLayout：紧凑但不挤压，字段可拉伸抢宽度
QFormLayout* makeFormLayout(QWidget* parent) {
    auto* f = new QFormLayout(parent);
    f->setContentsMargins(12, 20, 12, 12);
    f->setHorizontalSpacing(kFormHSpacing);
    f->setVerticalSpacing(kFormVSpacing);
    f->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    f->setRowWrapPolicy(QFormLayout::DontWrapRows);
    f->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    return f;
}

} // namespace

// ───────────────────────────────────────────────────────────────────────
//  Constructor / destructor
// ───────────────────────────────────────────────────────────────────────

BuildVprDatabaseDialog::BuildVprDatabaseDialog(const QString& submapId,
                                                const QString& outputDir,
                                                const QString& defaultImageCacheDir,
                                                const BuildVprParams& initial,
                                                QWidget* parent)
    : QDialog(parent),
      submapId_(submapId),
      outputDir_(outputDir),
      defaultImageCacheDir_(defaultImageCacheDir) {
    setWindowTitle(QString("构建 VPR 数据库 — %1").arg(submapId));
    setMinimumSize(960, 820);
    resize(1040, 880);
    buildUI();
    populateFromParams(initial);
}

BuildVprDatabaseDialog::~BuildVprDatabaseDialog() = default;

// ───────────────────────────────────────────────────────────────────────
//  UI construction
// ───────────────────────────────────────────────────────────────────────

QWidget* BuildVprDatabaseDialog::buildS20Tab() {
    auto* page = new QWidget();
    auto* root = new QVBoxLayout(page);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(10);

    auto makeBrowseRow = [&](QLineEdit*& edit, const char* placeholder,
                              auto&& slot) {
        auto* row = new QHBoxLayout();
        row->setSpacing(8);
        edit = new QLineEdit();
        edit->setPlaceholderText(placeholder);
        edit->setMinimumWidth(kLineEditMinWidth);
        edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        auto* btn = new QPushButton("浏览...");
        btn->setFixedWidth(kBrowseBtnWidth);
        row->addWidget(edit, /*stretch=*/1);
        row->addWidget(btn);
        connect(btn, &QPushButton::clicked, this, slot);
        return row;
    };

    auto* inputBox = new QGroupBox("输入 / 话题");
    auto* form = makeFormLayout(inputBox);

    form->addRow("Bag 文件 (.bag):",
        makeBrowseRow(editS20Bag_, "选择 ROS1 .bag",
                      &BuildVprDatabaseDialog::onBrowseS20Bag));
    form->addRow("位姿文件 (ImgPose.txt):",
        makeBrowseRow(editPose_, "选择 ImgPose.txt",
                      &BuildVprDatabaseDialog::onBrowsePose));

    editS20Topic_ = new QLineEdit();
    editS20Topic_->setPlaceholderText("/camera_agent/img_left/compressed");
    editS20Topic_->setMinimumWidth(kLineEditMinWidth);
    form->addRow("图像话题:", editS20Topic_);

    form->addRow("摄像头内参 (.yaml, 可选):",
        makeBrowseRow(editCalib_, "留空使用 PolyFisheye S20 默认内参",
                      &BuildVprDatabaseDialog::onBrowseCalibration));
    root->addWidget(inputBox);

    auto* paramsBox = new QGroupBox("S20 参数");
    auto* pform = makeFormLayout(paramsBox);

    chkUndistort_ = new QCheckBox("启用鱼眼去畸变 (POLYFISHEYE → perspective)");
    pform->addRow(chkUndistort_);

    auto* fisheyeRow = new QHBoxLayout();
    fisheyeRow->setSpacing(8);
    spinOutW_ = new QSpinBox();  spinOutW_->setRange(64, 8192);
    spinOutW_->setMinimumWidth(90);
    spinOutH_ = new QSpinBox();  spinOutH_->setRange(64, 8192);
    spinOutH_->setMinimumWidth(90);
    spinFov_  = new QDoubleSpinBox();
    spinFov_->setRange(1.0, 179.0);
    spinFov_->setDecimals(1);
    spinFov_->setMinimumWidth(90);
    fisheyeRow->addWidget(new QLabel("宽:"));
    fisheyeRow->addWidget(spinOutW_);
    fisheyeRow->addSpacing(12);
    fisheyeRow->addWidget(new QLabel("高:"));
    fisheyeRow->addWidget(spinOutH_);
    fisheyeRow->addSpacing(12);
    fisheyeRow->addWidget(new QLabel("FOV°:"));
    fisheyeRow->addWidget(spinFov_);
    fisheyeRow->addStretch(1);
    pform->addRow("输出尺寸 / FOV:", fisheyeRow);
    root->addWidget(paramsBox);

    connect(chkUndistort_, &QCheckBox::toggled, this,
            &BuildVprDatabaseDialog::onUndistortToggled);

    root->addStretch(1);
    return page;
}

QWidget* BuildVprDatabaseDialog::buildR1proTab() {
    auto* page = new QWidget();
    auto* root = new QVBoxLayout(page);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(10);

    auto* inputBox = new QGroupBox("输入 / 话题");
    auto* form = makeFormLayout(inputBox);

    {
        auto* row = new QHBoxLayout();
        row->setSpacing(8);
        editR1proBag_ = new QLineEdit();
        editR1proBag_->setPlaceholderText(
            "选择 ROS2 bag 目录（含 metadata.yaml + *.db3）或直接选 *.db3 文件");
        editR1proBag_->setMinimumWidth(kLineEditMinWidth);
        editR1proBag_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        auto* btn = new QPushButton("浏览...");
        btn->setFixedWidth(kBrowseBtnWidth);
        row->addWidget(editR1proBag_, /*stretch=*/1);
        row->addWidget(btn);
        connect(btn, &QPushButton::clicked, this,
                &BuildVprDatabaseDialog::onBrowseR1proBag);
        form->addRow("Bag 目录 / .db3:", row);
    }

    editPoseTopic_ = new QLineEdit();
    editPoseTopic_->setPlaceholderText("/Odometry");
    editPoseTopic_->setMinimumWidth(kLineEditMinWidth);
    form->addRow("位姿话题 (Odometry):", editPoseTopic_);

    editR1proTopic_ = new QLineEdit();
    editR1proTopic_->setPlaceholderText(
        "/hdas/camera_head/left_raw/image_raw_color/compressed");
    editR1proTopic_->setMinimumWidth(kLineEditMinWidth);
    form->addRow("图像话题 (CompressedImage):", editR1proTopic_);
    root->addWidget(inputBox);

    auto* paramsBox = new QGroupBox("R1Pro 参数 (PinHole 去畸变 + 中心裁剪)");
    auto* pform = makeFormLayout(paramsBox);

    auto* note = new QLabel(
        QStringLiteral(
            "R1Pro 路径自动应用 PinHoleUndistorter (R1Pro 默认内参)，并把去畸变后"
            "的图像中心裁剪到下方指定尺寸。\n\n"
            "裁剪尺寸必须与运行时 vpr_bootstrap._VPR_QUERY_TARGET_W/H 一致，"
            "否则建库图与在线查询图不在同一几何空间，会显著影响匹配精度。"));
    note->setStyleSheet("color: #666; font-size: 11px;");
    note->setWordWrap(true);
    note->setMinimumHeight(48);
    pform->addRow(note);

    auto* cropRow = new QHBoxLayout();
    cropRow->setSpacing(8);
    spinCropW_ = new QSpinBox(); spinCropW_->setRange(64, 8192);
    spinCropW_->setMinimumWidth(90);
    spinCropH_ = new QSpinBox(); spinCropH_->setRange(64, 8192);
    spinCropH_->setMinimumWidth(90);
    cropRow->addWidget(new QLabel("宽:"));
    cropRow->addWidget(spinCropW_);
    cropRow->addSpacing(12);
    cropRow->addWidget(new QLabel("高:"));
    cropRow->addWidget(spinCropH_);
    cropRow->addStretch(1);
    pform->addRow("中心裁剪尺寸:", cropRow);
    root->addWidget(paramsBox);

    root->addStretch(1);
    return page;
}

void BuildVprDatabaseDialog::buildUI() {
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(10);

    // ── 顶部 tab：数据来源 ─────────────────────────────────────────
    tabs_ = new QTabWidget();
    tabs_->setDocumentMode(false);
    tabs_->addTab(buildS20Tab(),   "S20 (ROS1 bag)");
    tabs_->addTab(buildR1proTab(), "R1Pro (ROS2 bag)");
    // tab 区不要被日志撑得太小：它的内容是 fixed 高度，给一个温和的最小值
    tabs_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    rootLayout->addWidget(tabs_);

    // ── 公共参数 ───────────────────────────────────────────────────
    auto* sharedBox = new QGroupBox("公共参数");
    auto* sform = makeFormLayout(sharedBox);

    spinDbDist_ = new QDoubleSpinBox();
    spinDbDist_->setRange(0.01, 100.0);
    spinDbDist_->setDecimals(2);
    spinDbDist_->setSingleStep(0.1);
    spinDbDist_->setSuffix(" m");
    spinDbDist_->setMinimumWidth(110);
    sform->addRow("数据库采样间距 (db_distance):", spinDbDist_);

    cmbMethod_ = new QComboBox();
    cmbMethod_->addItems({"salad-indoor", "megaloc"});
    cmbMethod_->setMinimumWidth(180);
    sform->addRow("VPR 模型:", cmbMethod_);

    cmbDevice_ = new QComboBox();
    cmbDevice_->addItems({"cuda", "cpu"});
    cmbDevice_->setMinimumWidth(120);
    sform->addRow("设备:", cmbDevice_);

    auto* perfRow = new QHBoxLayout();
    perfRow->setSpacing(8);
    spinBatch_   = new QSpinBox(); spinBatch_->setRange(1, 64);
    spinBatch_->setMinimumWidth(80);
    spinWorkers_ = new QSpinBox(); spinWorkers_->setRange(0, 64);
    spinWorkers_->setMinimumWidth(80);
    perfRow->addWidget(new QLabel("batch:"));
    perfRow->addWidget(spinBatch_);
    perfRow->addSpacing(16);
    perfRow->addWidget(new QLabel("workers:"));
    perfRow->addWidget(spinWorkers_);
    perfRow->addStretch(1);
    sform->addRow("性能:", perfRow);

    QFont mono = font();
    mono.setFamily("monospace");

    editOutputDir_ = new QLineEdit(outputDir_);
    editOutputDir_->setReadOnly(true);
    editOutputDir_->setFont(mono);
    editOutputDir_->setMinimumWidth(kLineEditMinWidth);
    editOutputDir_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    editOutputDir_->setToolTip(
        QStringLiteral("数据库文件 (.npy / .npz / .txt / .log) 的存放路径。\n"
                       "由 MapEditor 根据工程目录与 submap ID 自动决定。"));
    sform->addRow("数据库目录:", editOutputDir_);

    auto* cacheRow = new QHBoxLayout();
    cacheRow->setSpacing(8);
    editImageCacheDir_ = new QLineEdit();
    editImageCacheDir_->setFont(mono);
    editImageCacheDir_->setMinimumWidth(kLineEditMinWidth);
    editImageCacheDir_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    editImageCacheDir_->setToolTip(
        QStringLiteral("提取出的 JPG 缓存目录。\n"
                       "默认放在用户目录下的 ~/.vpr/<工程名>/<submapId>/，\n"
                       "这样工程目录里只保留检测所需的描述符数据。\n"
                       "如需放到其他磁盘可在此修改。"));
    auto* btnCache = new QPushButton("浏览...");
    btnCache->setFixedWidth(kBrowseBtnWidth);
    cacheRow->addWidget(editImageCacheDir_, /*stretch=*/1);
    cacheRow->addWidget(btnCache);
    connect(btnCache, &QPushButton::clicked, this,
            &BuildVprDatabaseDialog::onBrowseImageCache);
    sform->addRow("图片缓存目录:", cacheRow);

    rootLayout->addWidget(sharedBox);

    // ── 日志 ───────────────────────────────────────────────────────
    auto* logBox = new QGroupBox("运行日志");
    auto* logLayout = new QVBoxLayout(logBox);
    logLayout->setContentsMargins(10, 18, 10, 10);
    logView_ = new QPlainTextEdit();
    logView_->setReadOnly(true);
    logView_->setFont(QFont("monospace"));
    logView_->setMinimumHeight(160);
    logView_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    logLayout->addWidget(logView_);
    // 日志区拿剩余的垂直空间；tabs / sharedBox 保持各自的 sizeHint
    rootLayout->addWidget(logBox, /*stretch=*/1);

    // ── 按钮 ───────────────────────────────────────────────────────
    auto* btnRow = new QHBoxLayout();
    btnRow->setSpacing(10);
    btnRun_   = new QPushButton("▶ 运行");
    btnStop_  = new QPushButton("■ 停止");
    btnClose_ = new QPushButton("关闭");
    for (QPushButton* b : {btnRun_, btnStop_, btnClose_}) {
        b->setMinimumWidth(kActionBtnMinW);
        b->setMinimumHeight(30);
    }
    btnStop_->setEnabled(false);
    btnRow->addStretch(1);
    btnRow->addWidget(btnRun_);
    btnRow->addWidget(btnStop_);
    btnRow->addWidget(btnClose_);
    rootLayout->addLayout(btnRow);

    connect(btnRun_,   &QPushButton::clicked, this, &BuildVprDatabaseDialog::onRun);
    connect(btnStop_,  &QPushButton::clicked, this, &BuildVprDatabaseDialog::onStop);
    connect(btnClose_, &QPushButton::clicked, this, &QDialog::accept);
}

void BuildVprDatabaseDialog::populateFromParams(const BuildVprParams& p) {
    // S20 tab
    editS20Bag_->setText(p.s20BagPath);
    editPose_->setText(p.poseFile);
    editS20Topic_->setText(p.s20ImageTopic);
    editCalib_->setText(p.calibrationFile);
    chkUndistort_->setChecked(p.undistortFisheye);
    spinOutW_->setValue(p.outWidth);
    spinOutH_->setValue(p.outHeight);
    spinFov_->setValue(p.fov);
    onUndistortToggled(p.undistortFisheye);

    // R1Pro tab
    editR1proBag_->setText(p.r1proBagPath);
    editPoseTopic_->setText(p.poseTopic);
    editR1proTopic_->setText(p.r1proImageTopic);
    spinCropW_->setValue(p.cropWidth);
    spinCropH_->setValue(p.cropHeight);

    // 公共
    spinDbDist_->setValue(p.dbDistance);
    if (int idx = cmbMethod_->findText(p.method); idx >= 0) cmbMethod_->setCurrentIndex(idx);
    if (int idx = cmbDevice_->findText(p.device); idx >= 0) cmbDevice_->setCurrentIndex(idx);
    spinBatch_->setValue(p.batchSize);
    spinWorkers_->setValue(p.numWorkers);
    editImageCacheDir_->setText(
        p.imageCacheDir.isEmpty() ? defaultImageCacheDir_ : p.imageCacheDir
    );

    // 决定激活的 tab
    if (p.source == kSourceRos2R1Pro) tabs_->setCurrentIndex(kTabIdxR1Pro);
    else                              tabs_->setCurrentIndex(kTabIdxS20);
}

BuildVprParams BuildVprDatabaseDialog::currentParams() const {
    BuildVprParams p;
    p.source = activeSource();

    // S20 tab
    p.s20BagPath       = editS20Bag_->text().trimmed();
    p.poseFile         = editPose_->text().trimmed();
    p.s20ImageTopic    = editS20Topic_->text().trimmed();
    p.calibrationFile  = editCalib_->text().trimmed();
    p.undistortFisheye = chkUndistort_->isChecked();
    p.outWidth         = spinOutW_->value();
    p.outHeight        = spinOutH_->value();
    p.fov              = spinFov_->value();

    // R1Pro tab
    p.r1proBagPath     = editR1proBag_->text().trimmed();
    p.poseTopic        = editPoseTopic_->text().trimmed();
    p.r1proImageTopic  = editR1proTopic_->text().trimmed();
    p.cropWidth        = spinCropW_->value();
    p.cropHeight       = spinCropH_->value();

    // 公共
    p.dbDistance       = spinDbDist_->value();
    p.method           = cmbMethod_->currentText();
    p.device           = cmbDevice_->currentText();
    p.batchSize        = spinBatch_->value();
    p.numWorkers       = spinWorkers_->value();
    p.imageCacheDir    = editImageCacheDir_->text().trimmed();
    return p;
}

QString BuildVprDatabaseDialog::activeSource() const {
    return tabs_->currentIndex() == kTabIdxR1Pro
        ? QString::fromLatin1(kSourceRos2R1Pro)
        : QString::fromLatin1(kSourceRos1);
}

// ───────────────────────────────────────────────────────────────────────
//  Browsing / validation
// ───────────────────────────────────────────────────────────────────────

void BuildVprDatabaseDialog::onBrowseS20Bag() {
    QString start = QFileInfo(editS20Bag_->text()).absoluteDir().absolutePath();
    QString path = QFileDialog::getOpenFileName(this, "选择 ROS1 bag",
        start, "ROS1 Bag (*.bag);;All Files (*)");
    if (path.isEmpty()) return;
    editS20Bag_->setText(path);
    // 自动猜同目录下的 ImgPose.txt
    if (editPose_->text().trimmed().isEmpty()) {
        QString guess = QFileInfo(path).absoluteDir().absoluteFilePath("ImgPose.txt");
        if (QFile::exists(guess)) editPose_->setText(guess);
    }
}

void BuildVprDatabaseDialog::onBrowsePose() {
    QString start = QFileInfo(editPose_->text()).absoluteDir().absolutePath();
    if (start.isEmpty() || start == ".") {
        start = QFileInfo(editS20Bag_->text()).absoluteDir().absolutePath();
    }
    QString path = QFileDialog::getOpenFileName(this, "选择 ImgPose.txt",
        start, "Pose Files (*.txt);;All Files (*)");
    if (!path.isEmpty()) editPose_->setText(path);
}

void BuildVprDatabaseDialog::onBrowseCalibration() {
    QString start = QFileInfo(editCalib_->text()).absoluteDir().absolutePath();
    QString path = QFileDialog::getOpenFileName(this, "选择标定 YAML",
        start, "YAML (*.yaml *.yml);;All Files (*)");
    if (!path.isEmpty()) editCalib_->setText(path);
}

void BuildVprDatabaseDialog::onBrowseR1proBag() {
    // 默认目录：之前选过的 bag 所在的同级；否则用户家目录
    QString seed = editR1proBag_->text().trimmed();
    QFileInfo seedFi(seed);
    QString start = seedFi.isDir() ? seedFi.absoluteFilePath()
                                    : seedFi.absolutePath();
    if (start.isEmpty() || start == ".") start = QDir::homePath();

    QString dir = QFileDialog::getExistingDirectory(this,
        "选择 R1Pro ROS2 bag 目录 (含 metadata.yaml + *.db3)",
        start, QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) editR1proBag_->setText(dir);
}

void BuildVprDatabaseDialog::onBrowseImageCache() {
    QString start = editImageCacheDir_->text().trimmed();
    if (start.isEmpty()) start = defaultImageCacheDir_;
    QString dir = QFileDialog::getExistingDirectory(this,
        "选择图片缓存目录", start,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) editImageCacheDir_->setText(dir);
}

void BuildVprDatabaseDialog::onUndistortToggled(bool enabled) {
    spinOutW_->setEnabled(enabled);
    spinOutH_->setEnabled(enabled);
    spinFov_->setEnabled(enabled);
    editCalib_->setEnabled(enabled);
}

bool BuildVprDatabaseDialog::validate(QString* err) const {
    auto fail = [&](const QString& s) { if (err) *err = s; return false; };

    if (outputDir_.isEmpty())
        return fail("数据库目录未设置（请先保存工程）");

    if (editImageCacheDir_->text().trimmed().isEmpty())
        return fail("请填写图片缓存目录");

    if (activeSource() == QString::fromLatin1(kSourceRos2R1Pro)) {
        // ── R1Pro ──
        QString bag = editR1proBag_->text().trimmed();
        if (bag.isEmpty())
            return fail("请选择 R1Pro ROS2 bag 目录");
        QFileInfo bagFi(bag);
        if (!bagFi.exists())
            return fail("Bag 路径不存在: " + bag);
        // 接受目录或 .db3 文件；vpr_preprocess.py 内部会把 .db3 转为父目录
        if (bagFi.isFile() && !bag.endsWith(".db3", Qt::CaseInsensitive))
            return fail("Bag 必须是 ROS2 bag 目录或 .db3 文件");

        if (editPoseTopic_->text().trimmed().isEmpty())
            return fail("请填写位姿话题 (Odometry topic)");
        if (editR1proTopic_->text().trimmed().isEmpty())
            return fail("请填写图像话题 (CompressedImage topic)");
        if (spinCropW_->value() <= 0 || spinCropH_->value() <= 0)
            return fail("中心裁剪尺寸必须为正整数");
    } else {
        // ── S20 ──
        QString bag = editS20Bag_->text().trimmed();
        if (bag.isEmpty())              return fail("请选择 ROS1 bag 文件");
        if (!QFile::exists(bag))        return fail("Bag 文件不存在: " + bag);

        QString pose = editPose_->text().trimmed();
        if (pose.isEmpty())             return fail("请选择位姿文件 (ImgPose.txt)");
        if (!QFile::exists(pose))       return fail("位姿文件不存在: " + pose);

        if (editS20Topic_->text().trimmed().isEmpty())
            return fail("请填写图像话题");

        if (chkUndistort_->isChecked()) {
            QString calib = editCalib_->text().trimmed();
            if (!calib.isEmpty() && !QFile::exists(calib))
                return fail("标定文件不存在: " + calib);
        }
    }
    return true;
}

QStringList BuildVprDatabaseDialog::buildArgList(const QString& scriptPath) const {
    QStringList args;
    args << scriptPath
         << "--source"          << activeSource()
         << "--output_dir"      << outputDir_
         << "--image_cache_dir" << editImageCacheDir_->text().trimmed()
         << "--db_distance"     << QString::number(spinDbDist_->value(), 'f', 3)
         << "--method"          << cmbMethod_->currentText()
         << "--device"          << cmbDevice_->currentText()
         << "--batch_size"      << QString::number(spinBatch_->value())
         << "--num_workers"     << QString::number(spinWorkers_->value());

    if (activeSource() == QString::fromLatin1(kSourceRos2R1Pro)) {
        args << "--bag_path"    << editR1proBag_->text().trimmed()
             << "--pose_topic"  << editPoseTopic_->text().trimmed()
             << "--image_topic" << editR1proTopic_->text().trimmed()
             << "--crop_width"  << QString::number(spinCropW_->value())
             << "--crop_height" << QString::number(spinCropH_->value());
    } else {
        args << "--bag_path"    << editS20Bag_->text().trimmed()
             << "--pose_file"   << editPose_->text().trimmed()
             << "--image_topic" << editS20Topic_->text().trimmed();

        if (chkUndistort_->isChecked()) {
            args << "--undistort_fisheye"
                 << "--out_width"  << QString::number(spinOutW_->value())
                 << "--out_height" << QString::number(spinOutH_->value())
                 << "--fov"        << QString::number(spinFov_->value(), 'f', 1);
            QString calib = editCalib_->text().trimmed();
            if (!calib.isEmpty()) args << "--calibration_file" << calib;
        }
    }
    return args;
}

// ───────────────────────────────────────────────────────────────────────
//  Path resolution
// ───────────────────────────────────────────────────────────────────────

// VPR 模块已搬到 NaviSkill/vpr/ 下面。下面的两个 findVpr*() 助手从
// MapEditor 可执行文件所在目录向上寻祖，命中第一个有效路径。
// 仍保留 vpr/vpr-pipeline-v1/... 作为兼容候选，便于在旧 checkout 上运行。
QString BuildVprDatabaseDialog::findVprPython() {
    QString v = qEnvironmentVariable("VPR_PYTHON");
    if (!v.isEmpty()) return v;
    const QStringList relCandidates = {
        "/NaviSkill/vpr/.venv/bin/python",
        "/vpr/vpr-pipeline-v1/.venv/bin/python",  // legacy
    };
    QDir d(QCoreApplication::applicationDirPath());
    forever {
        for (const QString& rel : relCandidates) {
            QString cand = d.absolutePath() + rel;
            if (QFile::exists(cand)) return cand;
        }
        if (!d.cdUp()) break;
        if (d.absolutePath() == "/") break;
    }
    return "python3"; // 兜底
}

QString BuildVprDatabaseDialog::findVprScript() {
    QString v = qEnvironmentVariable("VPR_PREPROCESS_SCRIPT");
    if (!v.isEmpty()) return v;
    const QStringList relCandidates = {
        "/NaviSkill/vpr/vpr_preprocess.py",
        "/vpr/vpr-pipeline-v1/vpr_preprocess.py",  // legacy
    };
    QDir d(QCoreApplication::applicationDirPath());
    forever {
        for (const QString& rel : relCandidates) {
            QString cand = d.absolutePath() + rel;
            if (QFile::exists(cand)) return cand;
        }
        if (!d.cdUp()) break;
        if (d.absolutePath() == "/") break;
    }
    return QString();
}

QString BuildVprDatabaseDialog::stripAnsi(const QString& s) {
    static const QRegularExpression re("\x1B\\[[0-9;?]*[A-Za-z]");
    QString out = s;
    out.remove(re);
    return out;
}

// ───────────────────────────────────────────────────────────────────────
//  Run / stop
// ───────────────────────────────────────────────────────────────────────

void BuildVprDatabaseDialog::appendLog(const QString& text) {
    logView_->moveCursor(QTextCursor::End);
    logView_->insertPlainText(text);
    logView_->moveCursor(QTextCursor::End);
}

void BuildVprDatabaseDialog::onRun() {
    if (process_ && process_->state() != QProcess::NotRunning) return;

    QString err;
    if (!validate(&err)) {
        QMessageBox::warning(this, "参数无效", err);
        return;
    }

    QString script = findVprScript();
    if (script.isEmpty()) {
        QMessageBox::critical(this, "找不到 vpr_preprocess.py",
            "未能定位 vpr_preprocess.py。\n"
            "请设置环境变量 VPR_PREPROCESS_SCRIPT 指向脚本路径，"
            "或将仓库放在 …/Navi/NaviSkill/vpr/ 下。");
        return;
    }
    QString python = findVprPython();

    if (!QDir().mkpath(outputDir_)) {
        QMessageBox::critical(this, "错误", "无法创建输出目录:\n" + outputDir_);
        return;
    }

    QStringList args = buildArgList(script);
    logView_->clear();
    appendLog(QString("[source] %1\n").arg(activeSource()));
    appendLog(QString("$ %1 %2\n\n").arg(python, args.join(' ')));

    process_ = new QProcess(this);
    process_->setProcessChannelMode(QProcess::MergedChannels);

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("NO_COLOR", "1");          // 抑制 loguru 默认颜色
    env.insert("PYTHONUNBUFFERED", "1");  // 让 Python 输出不缓存，日志实时
    process_->setProcessEnvironment(env);

    connect(process_, &QProcess::readyReadStandardOutput,
            this, &BuildVprDatabaseDialog::onProcessOutput);
    connect(process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &BuildVprDatabaseDialog::onProcessFinished);

    finishedOk_ = false;
    btnRun_->setEnabled(false);
    btnStop_->setEnabled(true);
    btnClose_->setEnabled(false);
    tabs_->setEnabled(false);   // 运行中禁止切换 tab

    process_->start(python, args);
    if (!process_->waitForStarted(5000)) {
        appendLog("[error] 进程启动失败: " + process_->errorString() + "\n");
        btnRun_->setEnabled(true);
        btnStop_->setEnabled(false);
        btnClose_->setEnabled(true);
        tabs_->setEnabled(true);
        process_->deleteLater();
        process_ = nullptr;
    }
}

void BuildVprDatabaseDialog::onStop() {
    if (!process_ || process_->state() == QProcess::NotRunning) return;
    if (QMessageBox::question(this, "停止构建",
            "确定要终止当前 VPR 构建任务吗？") != QMessageBox::Yes) {
        return;
    }
    process_->terminate();
    if (!process_->waitForFinished(3000)) process_->kill();
}

void BuildVprDatabaseDialog::onProcessOutput() {
    if (!process_) return;
    QByteArray data = process_->readAllStandardOutput();
    appendLog(stripAnsi(QString::fromUtf8(data)));
}

void BuildVprDatabaseDialog::onProcessFinished(int exitCode, int exitStatus) {
    finishedOk_ = (exitStatus == QProcess::NormalExit && exitCode == 0);
    appendLog(QString("\n[exit] code=%1 status=%2  %3\n")
        .arg(exitCode)
        .arg(exitStatus == QProcess::NormalExit ? "normal" : "crashed")
        .arg(finishedOk_ ? "✔ 成功" : "✘ 失败"));
    if (finishedOk_) {
        appendLog("[output] " + outputDir_ + "\n");
    }
    btnRun_->setEnabled(true);
    btnStop_->setEnabled(false);
    btnClose_->setEnabled(true);
    tabs_->setEnabled(true);
    if (process_) {
        process_->deleteLater();
        process_ = nullptr;
    }
}

void BuildVprDatabaseDialog::closeEvent(QCloseEvent* event) {
    if (process_ && process_->state() != QProcess::NotRunning) {
        auto answer = QMessageBox::question(this, "构建进行中",
            "VPR 构建仍在运行，确定要关闭并终止任务吗？");
        if (answer != QMessageBox::Yes) {
            event->ignore();
            return;
        }
        process_->terminate();
        if (!process_->waitForFinished(2000)) process_->kill();
    }
    QDialog::closeEvent(event);
}

} // namespace me
