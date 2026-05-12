#include "ui/VprDatabaseInfoDialog.h"

#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonValue>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QStringList>
#include <QUrl>
#include <QVBoxLayout>
#include <QWidget>

namespace me {

// ───────────────────────────────────────────────────────────────────────
//  Loading helpers
// ───────────────────────────────────────────────────────────────────────

VprDatabaseInfo loadVprDatabaseInfo(const QString& dbDir) {
    VprDatabaseInfo info;
    info.dbDir = QDir(dbDir).absolutePath();

    QFile f(info.dbDir + "/database_build_info.json");
    if (!f.open(QIODevice::ReadOnly)) return info;       // valid stays false

    QJsonParseError err{};
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (doc.isNull() || !doc.isObject()) return info;

    info.raw            = doc.object();
    info.valid          = true;
    info.builtAt        = info.raw.value("built_at").toString();
    info.method         = info.raw.value("method").toString();
    info.imageCacheDir  = info.raw.value("image_cache_dir").toString();
    info.numDatabase    = info.raw.value("num_database").toInt();
    info.descriptorDim  = info.raw.value("descriptor_dim").toInt();
    return info;
}

BuildVprParams toBuildVprParams(const VprDatabaseInfo& info) {
    BuildVprParams p;
    if (!info.valid) return p;
    const QJsonObject& j = info.raw;
    p.bagPath          = j.value("bag_path").toString();
    p.poseFile         = j.value("pose_file").toString();
    p.imageTopic       = j.value("image_topic").toString(p.imageTopic);
    p.calibrationFile  = j.value("calibration_file").toString();
    p.undistortFisheye = j.value("undistort_fisheye").toBool(p.undistortFisheye);
    p.dbDistance       = j.value("db_distance").toDouble(p.dbDistance);
    p.outWidth         = j.value("out_width").toInt(p.outWidth);
    p.outHeight        = j.value("out_height").toInt(p.outHeight);
    p.fov              = j.value("fov").toDouble(p.fov);
    p.method           = j.value("method").toString(p.method);
    p.device           = j.value("device").toString(p.device);
    p.batchSize        = j.value("batch_size").toInt(p.batchSize);
    p.numWorkers       = j.value("num_workers").toInt(p.numWorkers);
    p.imageCacheDir    = info.imageCacheDir;
    return p;
}

// ───────────────────────────────────────────────────────────────────────
//  Small formatting helpers
// ───────────────────────────────────────────────────────────────────────

namespace {

QString formatBytes(qint64 b) {
    if (b < 1024) return QString::number(b) + " B";
    double kb = b / 1024.0;
    if (kb < 1024.0) return QString::number(kb, 'f', 1) + " KB";
    double mb = kb / 1024.0;
    if (mb < 1024.0) return QString::number(mb, 'f', 2) + " MB";
    return QString::number(mb / 1024.0, 'f', 2) + " GB";
}

QLineEdit* makePathField(const QString& text) {
    auto* le = new QLineEdit(text);
    le->setReadOnly(true);
    QFont mono = le->font();
    mono.setFamily("monospace");
    le->setFont(mono);
    le->setCursorPosition(0);
    le->setToolTip(text);
    return le;
}

QString imageSizeString(const QJsonValue& v) {
    if (v.isNull() || !v.isArray() || v.toArray().isEmpty()) return "（原图）";
    QStringList parts;
    for (const auto& x : v.toArray()) parts << QString::number(x.toInt());
    return parts.join(" × ");
}

} // namespace

// ───────────────────────────────────────────────────────────────────────
//  Dialog
// ───────────────────────────────────────────────────────────────────────

VprDatabaseInfoDialog::VprDatabaseInfoDialog(const QString& submapId,
                                              const QString& dbDir,
                                              QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(QString("VPR 数据库信息 — %1").arg(submapId));
    setMinimumSize(720, 620);

    VprDatabaseInfo info = loadVprDatabaseInfo(dbDir);

    auto* outer = new QVBoxLayout(this);

    // 用 QScrollArea 包住内容区，避免 group 多时弹出溢出
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto* content = new QWidget();
    auto* root = new QVBoxLayout(content);
    scroll->setWidget(content);
    outer->addWidget(scroll, /*stretch=*/1);

    // ── 概览 ───────────────────────────────────────────────────────
    {
        auto* box = new QGroupBox("概览");
        auto* form = new QFormLayout(box);
        form->setContentsMargins(10, 18, 10, 10);
        form->addRow("数据库目录:", makePathField(info.dbDir));
        form->addRow("图片缓存:", makePathField(
            info.imageCacheDir.isEmpty() ? "（未记录）" : info.imageCacheDir));

        QString builtAtDisplay = info.builtAt;
        if (builtAtDisplay.isEmpty()) {
            QFileInfo metaFi(info.dbDir + "/database_meta.npz");
            if (metaFi.exists()) {
                builtAtDisplay = metaFi.lastModified()
                                     .toString("yyyy-MM-dd HH:mm:ss")
                                 + "（按文件 mtime）";
            } else {
                builtAtDisplay = "—";
            }
        }
        form->addRow("构建时间:", new QLabel(builtAtDisplay));

        if (!info.valid) {
            auto* note = new QLabel(
                QStringLiteral("⚠ 未找到 database_build_info.json，"
                               "详细构造参数无法显示。\n"
                               "重新构建可生成完整元信息。"));
            note->setStyleSheet("color: #c66; font-size: 11px;");
            note->setWordWrap(true);
            form->addRow(note);
        }
        root->addWidget(box);
    }

    // ── 特征 / 空间统计 ────────────────────────────────────────────
    const QJsonObject& j = info.raw;
    {
        auto* box = new QGroupBox("特征 / 空间统计");
        auto* form = new QFormLayout(box);
        form->setContentsMargins(10, 18, 10, 10);

        form->addRow("VPR 模型:",
            new QLabel(info.method.isEmpty() ? "—" : info.method));
        form->addRow("数据库图像数:",
            new QLabel(QString::number(info.numDatabase)));
        form->addRow("描述符维度:",
            new QLabel(QString::number(info.descriptorDim)));
        form->addRow("Resize 后尺寸:",
            new QLabel(info.valid ? imageSizeString(j.value("image_size")) : "—"));

        if (info.valid && j.contains("xy_min") && j.contains("xy_max")) {
            auto xyMin = j.value("xy_min").toArray();
            auto xyMax = j.value("xy_max").toArray();
            if (xyMin.size() == 2 && xyMax.size() == 2) {
                QString xy = QString("X: [%1, %2] m   Y: [%3, %4] m")
                    .arg(xyMin.at(0).toDouble(), 0, 'f', 2)
                    .arg(xyMax.at(0).toDouble(), 0, 'f', 2)
                    .arg(xyMin.at(1).toDouble(), 0, 'f', 2)
                    .arg(xyMax.at(1).toDouble(), 0, 'f', 2);
                form->addRow("XY 覆盖:", new QLabel(xy));
            }
            QString zr = QString("[%1, %2] m")
                .arg(j.value("z_min").toDouble(), 0, 'f', 2)
                .arg(j.value("z_max").toDouble(), 0, 'f', 2);
            form->addRow("Z 范围:", new QLabel(zr));
        }

        if (info.valid && j.contains("descriptor_extraction_seconds")) {
            double s = j.value("descriptor_extraction_seconds").toDouble();
            form->addRow("特征提取耗时:",
                new QLabel(QString::number(s, 'f', 1) + " s"));
        }
        root->addWidget(box);
    }

    // ── 构造参数（仅在 JSON 有效时显示）──────────────────────────
    if (info.valid) {
        auto* box = new QGroupBox("构造参数");
        auto* form = new QFormLayout(box);
        form->setContentsMargins(10, 18, 10, 10);

        form->addRow("Bag 文件:",   makePathField(j.value("bag_path").toString()));
        form->addRow("位姿文件:",   makePathField(j.value("pose_file").toString()));
        form->addRow("图像话题:",   makePathField(j.value("image_topic").toString()));
        form->addRow("db_distance:",
            new QLabel(QString::number(j.value("db_distance").toDouble(), 'f', 2) + " m"));

        bool undist = j.value("undistort_fisheye").toBool();
        QString uText = undist
            ? QString("启用 — %1 × %2, FOV %3°")
                .arg(j.value("out_width").toInt())
                .arg(j.value("out_height").toInt())
                .arg(j.value("fov").toDouble(), 0, 'f', 0)
            : QStringLiteral("关闭");
        form->addRow("鱼眼去畸变:", new QLabel(uText));

        QString calib = j.value("calibration_file").toString();
        form->addRow("标定文件:",
            makePathField(calib.isEmpty() ? "（默认 S20 内参）" : calib));

        form->addRow("推理设备:",  new QLabel(j.value("device").toString()));
        form->addRow("batch / workers:", new QLabel(
            QString("%1 / %2")
                .arg(j.value("batch_size").toInt())
                .arg(j.value("num_workers").toInt())));

        root->addWidget(box);
    }

    // ── 文件列表 ───────────────────────────────────────────────────
    {
        auto* box = new QGroupBox("文件");
        auto* form = new QFormLayout(box);
        form->setContentsMargins(10, 18, 10, 10);

        const QStringList files = {
            "database_descriptors.npy",
            "database_poses.npy",
            "database_image_paths.txt",
            "database_meta.npz",
            "database_build_info.json",
            "vpr_preprocess.log",
        };
        for (const auto& name : files) {
            QFileInfo fi(info.dbDir + "/" + name);
            QString sizeStr = fi.exists() ? formatBytes(fi.size()) : "—";
            form->addRow(name + ":", new QLabel(sizeStr));
        }
        root->addWidget(box);
    }

    root->addStretch(1);

    // ── 按钮 ───────────────────────────────────────────────────────
    auto* btnRow = new QHBoxLayout();
    auto* btnOpenDb = new QPushButton("📁 打开数据库目录");
    auto* btnOpenCache = new QPushButton("🗂  打开缓存目录");
    btnOpenCache->setEnabled(
        !info.imageCacheDir.isEmpty() && QFileInfo(info.imageCacheDir).exists()
    );
    auto* btnClose = new QPushButton("关闭");
    btnRow->addWidget(btnOpenDb);
    btnRow->addWidget(btnOpenCache);
    btnRow->addStretch();
    btnRow->addWidget(btnClose);
    outer->addLayout(btnRow);

    const QString dbDirCopy = info.dbDir;
    const QString cacheDirCopy = info.imageCacheDir;
    connect(btnOpenDb, &QPushButton::clicked, this, [dbDirCopy]() {
        QDesktopServices::openUrl(QUrl::fromLocalFile(dbDirCopy));
    });
    connect(btnOpenCache, &QPushButton::clicked, this, [cacheDirCopy]() {
        QDesktopServices::openUrl(QUrl::fromLocalFile(cacheDirCopy));
    });
    connect(btnClose, &QPushButton::clicked, this, &QDialog::accept);
}

} // namespace me
