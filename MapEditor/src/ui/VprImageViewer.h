#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  VprImageViewer — 非模态图片预览窗口（用于查看 VPR 紫色点对应的图片）
//   * 标题：<submapId>[<index>]  <文件名>
//   * 自动按窗口大小缩放（KeepAspectRatio）
//   * 多个实例可同时打开，互不阻塞
// ═══════════════════════════════════════════════════════════════════════

#include <QDialog>
#include <QString>

class QLabel;
class QPixmap;

namespace me {

class VprImageViewer : public QDialog {
    Q_OBJECT
public:
    VprImageViewer(const QString& submapId,
                   int index,
                   const QString& imagePath,
                   QWidget* parent = nullptr);

protected:
    void resizeEvent(QResizeEvent* e) override;

private:
    void updatePixmap();

    QLabel*   label_     = nullptr;
    QPixmap*  pixmap_    = nullptr;   // 原图，避免反复 IO
    QString   imagePath_;
};

} // namespace me
