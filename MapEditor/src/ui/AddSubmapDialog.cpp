#include "ui/AddSubmapDialog.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileInfo>
#include <QLabel>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

#include <pcl/io/pcd_io.h>

namespace me {

AddSubmapDialog::AddSubmapDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle("新增子地图 (Submap)");
    setMinimumWidth(480);

    auto* layout = new QVBoxLayout(this);

    // 基本信息
    auto* infoGroup = new QGroupBox("基本信息");
    auto* form = new QFormLayout(infoGroup);
    form->setContentsMargins(10, 20, 10, 10);
    editId_   = new QLineEdit();
    editId_->setPlaceholderText("例如: floor2");
    editName_ = new QLineEdit();
    editName_->setPlaceholderText("例如: 2楼");
    editDesc_ = new QLineEdit();
    editDesc_->setPlaceholderText("例如: Second floor / 2F");
    form->addRow("Submap ID:", editId_);
    form->addRow("显示名称:", editName_);
    form->addRow("描述:", editDesc_);
    layout->addWidget(infoGroup);

    // 文件导入
    auto* fileGroup = new QGroupBox("文件导入");
    auto* fileForm = new QFormLayout(fileGroup);
    fileForm->setContentsMargins(10, 20, 10, 10);

    auto* pcdRow = new QHBoxLayout();
    editPcdPath_ = new QLineEdit();
    editPcdPath_->setPlaceholderText("选择 PCD 文件 (必填)");
    auto* btnPcd = new QPushButton("浏览...");
    pcdRow->addWidget(editPcdPath_);
    pcdRow->addWidget(btnPcd);
    fileForm->addRow("PCD 文件:", pcdRow);

    auto* jsonRow = new QHBoxLayout();
    editJsonPath_ = new QLineEdit();
    editJsonPath_->setPlaceholderText("可选: 含语义点位的 JSON");
    auto* btnJson = new QPushButton("浏览...");
    jsonRow->addWidget(editJsonPath_);
    jsonRow->addWidget(btnJson);
    fileForm->addRow("JSON 文件:", jsonRow);

    layout->addWidget(fileGroup);

    // 提示
    auto* hint = new QLabel(
        "• 导入 PCD: 语义点位为空, Portal 为空, Transform 为恒等变换\n"
        "• 如有 JSON: 从 JSON 读取语义点位, Portal 为空, Transform 为恒等变换\n"
        "• 导入后可通过右键 → 对齐 Transform 来自动计算变换");
    hint->setStyleSheet("color: #888; font-size: 11px;");
    layout->addWidget(hint);

    // 按钮
    auto* btnLayout = new QHBoxLayout();
    auto* btnOk = new QPushButton("确定");
    auto* btnCancel = new QPushButton("取消");
    btnLayout->addStretch();
    btnLayout->addWidget(btnOk);
    btnLayout->addWidget(btnCancel);
    layout->addLayout(btnLayout);

    // 信号连接
    connect(btnPcd, &QPushButton::clicked, [this]() {
        QString path = QFileDialog::getOpenFileName(this, "选择 PCD 文件", "",
                                                     "PCD Files (*.pcd);;All Files (*)");
        if (!path.isEmpty()) editPcdPath_->setText(path);
    });

    connect(btnJson, &QPushButton::clicked, [this]() {
        QString path = QFileDialog::getOpenFileName(this, "选择 JSON 文件", "",
                                                     "JSON Files (*.json);;All Files (*)");
        if (!path.isEmpty()) editJsonPath_->setText(path);
    });

    connect(btnOk, &QPushButton::clicked, this, &AddSubmapDialog::onAccept);
    connect(btnCancel, &QPushButton::clicked, this, &QDialog::reject);
}

void AddSubmapDialog::onAccept() {
    QString id = editId_->text().trimmed();
    if (id.isEmpty()) {
        QMessageBox::warning(this, "错误", "请输入 Submap ID");
        return;
    }

    QString pcdPath = editPcdPath_->text().trimmed();
    if (pcdPath.isEmpty()) {
        QMessageBox::warning(this, "错误", "请选择 PCD 文件");
        return;
    }

    // 加载 PCD
    auto cloud = std::make_shared<CloudT>();
    if (pcl::io::loadPCDFile<PointT>(pcdPath.toStdString(), *cloud) == -1) {
        QMessageBox::warning(this, "错误", "无法加载 PCD 文件: " + pcdPath);
        return;
    }

    auto sm = std::make_shared<SubMap>(id);
    sm->setName(editName_->text().trimmed());
    sm->setDescription(editDesc_->text().trimmed());
    sm->setMapFile(QFileInfo(pcdPath).fileName());
    sm->setTransform(RigidTransform::identity());
    sm->setCloud(cloud);

    // 如果有 JSON, 尝试读取语义点位
    QString jsonPath = editJsonPath_->text().trimmed();
    if (!jsonPath.isEmpty()) {
        QFile jf(jsonPath);
        if (jf.open(QIODevice::ReadOnly)) {
            auto doc = QJsonDocument::fromJson(jf.readAll());
            auto root = doc.object();

            // 尝试读取 semantic_points (兼容多种格式)
            QJsonObject spObj;
            if (root.contains("semantic_points")) {
                spObj = root["semantic_points"].toObject();
            } else if (root.contains("submaps")) {
                // 从 multifloor.json 格式中读取第一个 submap 的语义点
                auto submaps = root["submaps"].toObject();
                for (auto it = submaps.begin(); it != submaps.end(); ++it) {
                    auto smData = it.value().toObject();
                    if (smData.contains("semantic_points")) {
                        spObj = smData["semantic_points"].toObject();
                        break;
                    }
                }
            }

            for (auto sit = spObj.begin(); sit != spObj.end(); ++sit) {
                auto spData = sit.value().toObject();
                SemanticPoint sp;
                sp.name = sit.key();
                sp.type = spData["type"].toString("single_pose");

                auto parsePose = [](const QJsonObject& obj) {
                    Pose p;
                    if (obj.contains("position")) {
                        auto pos = obj["position"].toObject();
                        p.position = {pos["x"].toDouble(), pos["y"].toDouble(), pos["z"].toDouble()};
                    }
                    if (obj.contains("orientation")) {
                        auto ori = obj["orientation"].toObject();
                        p.orientation = Eigen::Quaterniond(
                            ori["w"].toDouble(), ori["x"].toDouble(),
                            ori["y"].toDouble(), ori["z"].toDouble());
                    }
                    return p;
                };

                if (sp.type == "multi_pose" && spData.contains("poses")) {
                    for (const auto& pv : spData["poses"].toArray()) {
                        sp.poses.append(parsePose(pv.toObject()));
                    }
                } else if (spData.contains("pose")) {
                    sp.poses.append(parsePose(spData["pose"].toObject()));
                }

                if (spData.contains("need_inspect")) {
                    sp.needInspect = spData["need_inspect"].toBool();
                } else {
                    sp.needInspect = SemanticPoint::defaultNeedInspect(sp.type);
                }

                sm->addSemanticPoint(sp);
            }
        }
    }

    result_ = sm;
    accept();
}

} // namespace me
