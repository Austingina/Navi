#include "ui/VprImageViewer.h"

#include <QFileInfo>
#include <QVBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QResizeEvent>

namespace me {

VprImageViewer::VprImageViewer(const QString& submapId,
                               int index,
                               const QString& imagePath,
                               QWidget* parent)
    : QDialog(parent), imagePath_(imagePath) {
    setWindowTitle(QString("VPR  %1[%2]  %3")
                       .arg(submapId)
                       .arg(index)
                       .arg(QFileInfo(imagePath).fileName()));
    setAttribute(Qt::WA_DeleteOnClose);
    setModal(false);
    resize(720, 540);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);

    label_ = new QLabel(this);
    label_->setAlignment(Qt::AlignCenter);
    label_->setMinimumSize(200, 150);
    label_->setStyleSheet("background: #202020; color: #cccccc;");
    layout->addWidget(label_);

    pixmap_ = new QPixmap();
    if (!pixmap_->load(imagePath_)) {
        label_->setText(QString("无法加载图片:\n%1").arg(imagePath_));
        delete pixmap_;
        pixmap_ = nullptr;
    } else {
        updatePixmap();
    }
}

void VprImageViewer::resizeEvent(QResizeEvent* e) {
    QDialog::resizeEvent(e);
    updatePixmap();
}

void VprImageViewer::updatePixmap() {
    if (!pixmap_ || pixmap_->isNull() || !label_) return;
    QSize avail = label_->size();
    if (avail.width() < 10 || avail.height() < 10) return;
    label_->setPixmap(pixmap_->scaled(avail,
                                      Qt::KeepAspectRatio,
                                      Qt::SmoothTransformation));
}

} // namespace me
