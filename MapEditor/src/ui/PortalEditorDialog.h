#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  PortalEditorDialog — 空白新建传送门：选择语义点名称与目标 Submap
// ═══════════════════════════════════════════════════════════════════════

#include "core/SubMap.h"
#include <QDialog>
#include <QComboBox>

namespace me {

class PortalEditorDialog : public QDialog {
    Q_OBJECT
public:
    explicit PortalEditorDialog(SubMap* sm, const QStringList& allSubmapIds,
                                QWidget* parent = nullptr);

private slots:
    void onApply();

private:
    void setupUI();

    SubMap*     sm_ = nullptr;
    QComboBox*  nameCombo_  = nullptr;
    QComboBox*  targetCombo_ = nullptr;
};

} // namespace me
