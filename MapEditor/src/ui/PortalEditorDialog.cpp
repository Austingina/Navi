#include "ui/PortalEditorDialog.h"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <algorithm>

namespace me {

PortalEditorDialog::PortalEditorDialog(SubMap* sm, const QStringList& allSubmapIds,
                                       QWidget* parent)
    : QDialog(parent), sm_(sm) {
    setWindowTitle("新增传送门");
    setMinimumWidth(420);
    setupUI();

    if (!sm_) return;

    QStringList semNames;
    semNames.reserve(sm->semanticPoints().size());
    for (auto it = sm->semanticPoints().constBegin(); it != sm->semanticPoints().constEnd(); ++it) {
        semNames.append(it.key());
    }
    std::sort(semNames.begin(), semNames.end());
    nameCombo_->addItems(semNames);

    QStringList targets;
    for (const QString& id : allSubmapIds) {
        if (id != sm->id()) {
            targets.append(id);
        }
    }
    std::sort(targets.begin(), targets.end());
    targetCombo_->addItems(targets);

    nameCombo_->setCurrentIndex(-1);
    targetCombo_->setCurrentIndex(-1);
}

void PortalEditorDialog::setupUI() {
    auto* layout = new QVBoxLayout(this);

    auto* hint = new QLabel(
        "请选择本层子地图中已有语义点作为传送门位置，并选择要到达的目标 Submap。\n"
        "若下拉为空，请先在该子地图下添加语义点，或保证工程中还有其他子地图。");
    hint->setWordWrap(true);
    layout->addWidget(hint);

    auto* form = new QFormLayout();
    layout->addLayout(form);

    nameCombo_ = new QComboBox();
    nameCombo_->setMinimumContentsLength(16);
    targetCombo_ = new QComboBox();
    targetCombo_->setMinimumContentsLength(12);

    form->addRow("传送门名称 (语义点):", nameCombo_);
    form->addRow("目标 Submap:", targetCombo_);

    auto* btnRow = new QHBoxLayout();
    btnRow->addStretch();
    auto* btnApply = new QPushButton("应用");
    auto* btnCancel = new QPushButton("取消");
    btnRow->addWidget(btnApply);
    btnRow->addWidget(btnCancel);
    layout->addLayout(btnRow);

    connect(btnApply, &QPushButton::clicked, this, &PortalEditorDialog::onApply);
    connect(btnCancel, &QPushButton::clicked, this, &QDialog::reject);
}

void PortalEditorDialog::onApply() {
    if (!sm_) {
        reject();
        return;
    }

    QString portalName = nameCombo_->currentText().trimmed();
    QString targetSm   = targetCombo_->currentText().trimmed();

    if (portalName.isEmpty()) {
        QMessageBox::warning(this, "新增传送门", "请选择传送门对应的语义点名称。");
        return;
    }
    if (!sm_->hasSemanticPoint(portalName)) {
        QMessageBox::warning(this, "新增传送门",
            QString("语义点 \"%1\" 不存在，请先在子地图中添加该语义点。").arg(portalName));
        return;
    }
    if (targetSm.isEmpty()) {
        QMessageBox::warning(this, "新增传送门", "请选择目标 Submap。");
        return;
    }

    for (const auto& ep : sm_->portals()) {
        if (ep.portalName == portalName && ep.targetSubmap == targetSm) {
            QMessageBox::warning(this, "新增传送门",
                QString("已存在传送门 \"%1\" → %2，请勿重复添加。").arg(portalName, targetSm));
            return;
        }
    }

    Portal p;
    p.portalName   = portalName;
    p.targetSubmap = targetSm;
    sm_->addPortal(p);
    accept();
}

} // namespace me
