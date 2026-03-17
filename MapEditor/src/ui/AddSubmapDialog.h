#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  AddSubmapDialog — 新增子地图对话框
// ═══════════════════════════════════════════════════════════════════════

#include "core/SubMap.h"
#include <QDialog>
#include <QLineEdit>
#include <memory>

namespace me {

class AddSubmapDialog : public QDialog {
    Q_OBJECT
public:
    explicit AddSubmapDialog(QWidget* parent = nullptr);

    /// 返回创建的 SubMap (调用者接管所有权)
    std::shared_ptr<SubMap> result() const { return result_; }

private:
    void onAccept();

    QLineEdit* editId_ = nullptr;
    QLineEdit* editName_ = nullptr;
    QLineEdit* editDesc_ = nullptr;
    QLineEdit* editPcdPath_ = nullptr;
    QLineEdit* editJsonPath_ = nullptr;

    std::shared_ptr<SubMap> result_;
};

} // namespace me
