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
#include <QSpinBox>
#include <QTextCursor>
#include <QVBoxLayout>

namespace me {

// ───────────────────────────────────────────────────────────────────────
//  Constructor / destructor
// ───────────────────────────────────────────────────────────────────────

BuildVprDatabaseDialog::BuildVprDatabaseDialog(const QString& submapId,
                                                const QString& outputDir,
                                                const BuildVprParams& initial,
                                                QWidget* parent)
    : QDialog(parent), submapId_(submapId), outputDir_(outputDir) {
    setWindowTitle(QString("构建 VPR 数据库 — %1").arg(submapId));
    setMinimumSize(760, 640);
    buildUI();
    populateFromParams(initial);
}

BuildVprDatabaseDialog::~BuildVprDatabaseDialog() = default;

// ───────────────────────────────────────────────────────────────────────
//  UI construction
// ───────────────────────────────────────────────────────────────────────

void BuildVprDatabaseDialog::buildUI() {
    auto* root = new QVBoxLayout(this);

    // ── 输入 / 话题 ────────────────────────────────────────────────
    auto* inputBox = new QGroupBox("输入 / 话题");
    auto* form = new QFormLayout(inputBox);
    form->setContentsMargins(10, 18, 10, 10);

    auto makeBrowseRow = [&](QLineEdit*& edit, const char* placeholder,
                              auto&& slot) {
        auto* row = new QHBoxLayout();
        edit = new QLineEdit();
        edit->setPlaceholderText(placeholder);
        auto* btn = new QPushButton("浏览...");
        btn->setFixedWidth(80);
        row->addWidget(edit);
        row->addWidget(btn);
        connect(btn, &QPushButton::clicked, this, slot);
        return row;
    };

    form->addRow("Bag 文件 (.bag):",
        makeBrowseRow(editBag_, "选择 ROS1 .bag",
                      &BuildVprDatabaseDialog::onBrowseBag));
    form->addRow("位姿文件 (ImgPose.txt):",
        makeBrowseRow(editPose_, "选择 ImgPose.txt",
                      &BuildVprDatabaseDialog::onBrowsePose));

    editTopic_ = new QLineEdit();
    editTopic_->setPlaceholderText("/camera_agent/img_left/compressed");
    form->addRow("图像话题:", editTopic_);

    form->addRow("摄像头内参 (.yaml, 可选):",
        makeBrowseRow(editCalib_, "留空使用 PolyFisheye S20 默认内参",
                      &BuildVprDatabaseDialog::onBrowseCalibration));

    root->addWidget(inputBox);

    // ── 参数 ───────────────────────────────────────────────────────
    auto* paramsBox = new QGroupBox("参数");
    auto* pform = new QFormLayout(paramsBox);
    pform->setContentsMargins(10, 18, 10, 10);

    chkUndistort_ = new QCheckBox("启用鱼眼去畸变 (POLYFISHEYE → perspective)");
    pform->addRow(chkUndistort_);

    auto* fisheyeRow = new QHBoxLayout();
    spinOutW_ = new QSpinBox();  spinOutW_->setRange(64, 8192);
    spinOutH_ = new QSpinBox();  spinOutH_->setRange(64, 8192);
    spinFov_  = new QDoubleSpinBox();
    spinFov_->setRange(1.0, 179.0);
    spinFov_->setDecimals(1);
    fisheyeRow->addWidget(new QLabel("宽:"));
    fisheyeRow->addWidget(spinOutW_);
    fisheyeRow->addWidget(new QLabel("高:"));
    fisheyeRow->addWidget(spinOutH_);
    fisheyeRow->addWidget(new QLabel("FOV°:"));
    fisheyeRow->addWidget(spinFov_);
    fisheyeRow->addStretch();
    pform->addRow("输出尺寸 / FOV:", fisheyeRow);

    spinDbDist_ = new QDoubleSpinBox();
    spinDbDist_->setRange(0.01, 100.0);
    spinDbDist_->setDecimals(2);
    spinDbDist_->setSingleStep(0.1);
    spinDbDist_->setSuffix(" m");
    pform->addRow("数据库采样间距 (db_distance):", spinDbDist_);

    cmbMethod_ = new QComboBox();
    cmbMethod_->addItems({"salad-indoor", "megaloc"});
    pform->addRow("VPR 模型:", cmbMethod_);

    cmbDevice_ = new QComboBox();
    cmbDevice_->addItems({"cuda", "cpu"});
    pform->addRow("设备:", cmbDevice_);

    auto* perfRow = new QHBoxLayout();
    spinBatch_   = new QSpinBox(); spinBatch_->setRange(1, 64);
    spinWorkers_ = new QSpinBox(); spinWorkers_->setRange(0, 64);
    perfRow->addWidget(new QLabel("batch:"));
    perfRow->addWidget(spinBatch_);
    perfRow->addSpacing(12);
    perfRow->addWidget(new QLabel("workers:"));
    perfRow->addWidget(spinWorkers_);
    perfRow->addStretch();
    pform->addRow("性能:", perfRow);

    editOutputDir_ = new QLineEdit(outputDir_);
    editOutputDir_->setReadOnly(true);
    QFont mono = editOutputDir_->font();
    mono.setFamily("monospace");
    editOutputDir_->setFont(mono);
    pform->addRow("输出目录:", editOutputDir_);

    root->addWidget(paramsBox);

    // ── 日志 ───────────────────────────────────────────────────────
    auto* logBox = new QGroupBox("运行日志");
    auto* logLayout = new QVBoxLayout(logBox);
    logLayout->setContentsMargins(8, 14, 8, 8);
    logView_ = new QPlainTextEdit();
    logView_->setReadOnly(true);
    logView_->setFont(QFont("monospace"));
    logView_->setMinimumHeight(180);
    logLayout->addWidget(logView_);
    root->addWidget(logBox, /*stretch=*/1);

    // ── 按钮 ───────────────────────────────────────────────────────
    auto* btnRow = new QHBoxLayout();
    btnRun_   = new QPushButton("▶ 运行");
    btnStop_  = new QPushButton("■ 停止");
    btnClose_ = new QPushButton("关闭");
    btnStop_->setEnabled(false);
    btnRow->addStretch();
    btnRow->addWidget(btnRun_);
    btnRow->addWidget(btnStop_);
    btnRow->addWidget(btnClose_);
    root->addLayout(btnRow);

    connect(btnRun_,       &QPushButton::clicked, this, &BuildVprDatabaseDialog::onRun);
    connect(btnStop_,      &QPushButton::clicked, this, &BuildVprDatabaseDialog::onStop);
    connect(btnClose_,     &QPushButton::clicked, this, &QDialog::accept);
    connect(chkUndistort_, &QCheckBox::toggled,   this, &BuildVprDatabaseDialog::onUndistortToggled);
}

void BuildVprDatabaseDialog::populateFromParams(const BuildVprParams& p) {
    editBag_->setText(p.bagPath);
    editPose_->setText(p.poseFile);
    editTopic_->setText(p.imageTopic);
    editCalib_->setText(p.calibrationFile);
    chkUndistort_->setChecked(p.undistortFisheye);
    spinDbDist_->setValue(p.dbDistance);
    spinOutW_->setValue(p.outWidth);
    spinOutH_->setValue(p.outHeight);
    spinFov_->setValue(p.fov);
    if (int idx = cmbMethod_->findText(p.method); idx >= 0) cmbMethod_->setCurrentIndex(idx);
    if (int idx = cmbDevice_->findText(p.device); idx >= 0) cmbDevice_->setCurrentIndex(idx);
    spinBatch_->setValue(p.batchSize);
    spinWorkers_->setValue(p.numWorkers);
    onUndistortToggled(p.undistortFisheye);
}

BuildVprParams BuildVprDatabaseDialog::currentParams() const {
    BuildVprParams p;
    p.bagPath          = editBag_->text().trimmed();
    p.poseFile         = editPose_->text().trimmed();
    p.imageTopic       = editTopic_->text().trimmed();
    p.calibrationFile  = editCalib_->text().trimmed();
    p.undistortFisheye = chkUndistort_->isChecked();
    p.dbDistance       = spinDbDist_->value();
    p.outWidth         = spinOutW_->value();
    p.outHeight        = spinOutH_->value();
    p.fov              = spinFov_->value();
    p.method           = cmbMethod_->currentText();
    p.device           = cmbDevice_->currentText();
    p.batchSize        = spinBatch_->value();
    p.numWorkers       = spinWorkers_->value();
    return p;
}

// ───────────────────────────────────────────────────────────────────────
//  Browsing / validation
// ───────────────────────────────────────────────────────────────────────

void BuildVprDatabaseDialog::onBrowseBag() {
    QString start = QFileInfo(editBag_->text()).absoluteDir().absolutePath();
    QString path = QFileDialog::getOpenFileName(this, "选择 ROS1 bag",
        start, "ROS1 Bag (*.bag);;All Files (*)");
    if (path.isEmpty()) return;
    editBag_->setText(path);
    // 自动猜同目录下的 ImgPose.txt
    if (editPose_->text().trimmed().isEmpty()) {
        QString guess = QFileInfo(path).absoluteDir().absoluteFilePath("ImgPose.txt");
        if (QFile::exists(guess)) editPose_->setText(guess);
    }
}

void BuildVprDatabaseDialog::onBrowsePose() {
    QString start = QFileInfo(editPose_->text()).absoluteDir().absolutePath();
    if (start.isEmpty() || start == ".") {
        start = QFileInfo(editBag_->text()).absoluteDir().absolutePath();
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

void BuildVprDatabaseDialog::onUndistortToggled(bool enabled) {
    spinOutW_->setEnabled(enabled);
    spinOutH_->setEnabled(enabled);
    spinFov_->setEnabled(enabled);
    editCalib_->setEnabled(enabled);
}

bool BuildVprDatabaseDialog::validate(QString* err) const {
    auto fail = [&](const QString& s) { if (err) *err = s; return false; };

    if (outputDir_.isEmpty())
        return fail("输出目录未设置（请先保存工程）");

    QString bag = editBag_->text().trimmed();
    if (bag.isEmpty())              return fail("请选择 ROS1 bag 文件");
    if (!QFile::exists(bag))        return fail("Bag 文件不存在: " + bag);

    QString pose = editPose_->text().trimmed();
    if (pose.isEmpty())             return fail("请选择位姿文件 (ImgPose.txt)");
    if (!QFile::exists(pose))       return fail("位姿文件不存在: " + pose);

    if (editTopic_->text().trimmed().isEmpty())
        return fail("请填写图像话题");

    if (chkUndistort_->isChecked()) {
        QString calib = editCalib_->text().trimmed();
        if (!calib.isEmpty() && !QFile::exists(calib))
            return fail("标定文件不存在: " + calib);
    }
    return true;
}

QStringList BuildVprDatabaseDialog::buildArgList(const QString& scriptPath) const {
    QStringList args;
    args << scriptPath
         << "--bag_path"    << editBag_->text().trimmed()
         << "--pose_file"   << editPose_->text().trimmed()
         << "--image_topic" << editTopic_->text().trimmed()
         << "--output_dir"  << outputDir_
         << "--db_distance" << QString::number(spinDbDist_->value(), 'f', 3)
         << "--method"      << cmbMethod_->currentText()
         << "--device"      << cmbDevice_->currentText()
         << "--batch_size"  << QString::number(spinBatch_->value())
         << "--num_workers" << QString::number(spinWorkers_->value());

    if (chkUndistort_->isChecked()) {
        args << "--undistort_fisheye"
             << "--out_width"  << QString::number(spinOutW_->value())
             << "--out_height" << QString::number(spinOutH_->value())
             << "--fov"        << QString::number(spinFov_->value(), 'f', 1);
        QString calib = editCalib_->text().trimmed();
        if (!calib.isEmpty()) args << "--calibration_file" << calib;
    }
    return args;
}

// ───────────────────────────────────────────────────────────────────────
//  Path resolution
// ───────────────────────────────────────────────────────────────────────

QString BuildVprDatabaseDialog::findVprPython() {
    QString v = qEnvironmentVariable("VPR_PYTHON");
    if (!v.isEmpty()) return v;
    // 从可执行文件向上找 vpr/vpr-pipeline-v1/.venv/bin/python
    QDir d(QCoreApplication::applicationDirPath());
    forever {
        QString cand = d.absolutePath() + "/vpr/vpr-pipeline-v1/.venv/bin/python";
        if (QFile::exists(cand)) return cand;
        if (!d.cdUp()) break;
        if (d.absolutePath() == "/") break;
    }
    return "python3"; // 兜底
}

QString BuildVprDatabaseDialog::findVprScript() {
    QString v = qEnvironmentVariable("VPR_PREPROCESS_SCRIPT");
    if (!v.isEmpty()) return v;
    QDir d(QCoreApplication::applicationDirPath());
    forever {
        QString cand = d.absolutePath() + "/vpr/vpr-pipeline-v1/vpr_preprocess.py";
        if (QFile::exists(cand)) return cand;
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
            "或将仓库放在 …/Navi/vpr/vpr-pipeline-v1/ 下。");
        return;
    }
    QString python = findVprPython();

    if (!QDir().mkpath(outputDir_)) {
        QMessageBox::critical(this, "错误", "无法创建输出目录:\n" + outputDir_);
        return;
    }

    QStringList args = buildArgList(script);
    logView_->clear();
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

    process_->start(python, args);
    if (!process_->waitForStarted(5000)) {
        appendLog("[error] 进程启动失败: " + process_->errorString() + "\n");
        btnRun_->setEnabled(true);
        btnStop_->setEnabled(false);
        btnClose_->setEnabled(true);
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
