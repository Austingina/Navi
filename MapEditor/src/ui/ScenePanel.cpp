#include "ui/ScenePanel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QLabel>
#include <QMenu>
#include <QFileInfo>

namespace me {

ScenePanel::ScenePanel(QWidget* parent) : QWidget(parent) {
    setupUI();
}

void ScenePanel::setupUI() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    auto* toolbar = new QHBoxLayout();
    auto* btnOsm = new QPushButton("导入底图");
    auto* btnSlam = new QPushButton("导入SLAM地图");
    toolbar->addWidget(btnOsm);
    toolbar->addWidget(btnSlam);
    toolbar->addStretch();
    layout->addLayout(toolbar);
    connect(btnOsm,  &QPushButton::clicked, this, &ScenePanel::importOsmRequested);
    connect(btnSlam, &QPushButton::clicked, this, &ScenePanel::addSlamMapRequested);

    tree_ = new QTreeWidget();
    tree_->setHeaderLabels({"名称", "显示"});
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->header()->setStretchLastSection(false);
    tree_->setColumnWidth(1, 40);
    tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(tree_, &QTreeWidget::customContextMenuRequested, this, &ScenePanel::onContextMenu);
    connect(tree_, &QTreeWidget::itemClicked, this, &ScenePanel::onItemClicked);
    connect(tree_, &QTreeWidget::itemDoubleClicked, this, &ScenePanel::onItemDoubleClicked);
    connect(tree_, &QTreeWidget::itemChanged, this, &ScenePanel::onItemChanged);
    layout->addWidget(tree_);

    auto* hint = new QLabel("双击 SLAM 地图进入其内部 submap 编辑；勾选控制其正交投影叠加显示。");
    hint->setWordWrap(true);
    hint->setStyleSheet("color:#8a8f99;");
    layout->addWidget(hint);
}

void ScenePanel::rebuild(Scene* scene) {
    scene_ = scene;
    tree_->blockSignals(true);
    tree_->clear();

    QString osmLabel = "🗺️ OSM 底图 (base layer)";
    if (scene_ && scene_->hasOsm())
        osmLabel += QString(" — %1").arg(QFileInfo(scene_->osmSource()).fileName());
    else
        osmLabel += " — 未导入";
    auto* osmRoot = new QTreeWidgetItem(tree_, {osmLabel});
    osmRoot->setData(0, RoleType, TOsm);
    osmRoot->setExpanded(true);

    if (scene_) {
        for (int i = 0; i < scene_->maps().size(); ++i) {
            const auto& e = scene_->maps()[i];
            QString label = QString("🏢 %1 (%2)").arg(e.name.isEmpty() ? e.id : e.name, e.id);
            if (i == scene_->activeIndex()) label += " ⭐";
            if (e.anchor.valid)
                label += QString("  [lon %1, lat %2, %3°]")
                             .arg(e.anchor.lon, 0, 'f', 6)
                             .arg(e.anchor.lat, 0, 'f', 6)
                             .arg(e.anchor.headingDeg, 0, 'f', 1);
            auto* item = new QTreeWidgetItem({label});
            item->setData(0, RoleType, TSlam);
            item->setData(0, RoleId, e.id);
            item->setCheckState(1, e.orthoVisible ? Qt::Checked : Qt::Unchecked);
            osmRoot->addChild(item);
        }
    }

    tree_->blockSignals(false);
}

void ScenePanel::onItemClicked(QTreeWidgetItem* item, int) {
    if (!item) return;
    if (item->data(0, RoleType).toInt() == TSlam)
        emit slamMapSelected(item->data(0, RoleId).toString());
}

void ScenePanel::onItemDoubleClicked(QTreeWidgetItem* item, int) {
    if (!item) return;
    if (item->data(0, RoleType).toInt() == TSlam)
        emit slamMapActivated(item->data(0, RoleId).toString());
}

void ScenePanel::onItemChanged(QTreeWidgetItem* item, int column) {
    if (column != 1 || !item) return;
    if (item->data(0, RoleType).toInt() == TSlam)
        emit orthoVisibilityChanged(item->data(0, RoleId).toString(),
                                    item->checkState(1) == Qt::Checked);
}

void ScenePanel::onContextMenu(const QPoint& pos) {
    auto* item = tree_->itemAt(pos);
    QMenu menu;
    int type = item ? item->data(0, RoleType).toInt() : -1;
    if (type == TSlam) {
        QString id = item->data(0, RoleId).toString();
        menu.addAction("✅ 激活并编辑此地图", [this, id]() { emit slamMapActivated(id); });
        menu.addSeparator();
        menu.addAction("🏢 导入 SLAM 地图...", [this]() { emit addSlamMapRequested(); });
        menu.addAction("🗑️ 移除此地图", [this, id]() { emit removeSlamMapRequested(id); });
    } else {
        menu.addAction("🗺️ 导入 OSM/PBF 底图...", [this]() { emit importOsmRequested(); });
        menu.addAction("🏢 导入 SLAM 地图...", [this]() { emit addSlamMapRequested(); });
    }
    menu.exec(tree_->viewport()->mapToGlobal(pos));
}

} // namespace me
