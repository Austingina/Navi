#include "ui/ScenePanel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QLabel>
#include <QMenu>
#include <QInputDialog>
#include <QLineEdit>
#include <QFileInfo>

namespace me {

static QString sceneGlobalWpLabel(int i, const GlobalWaypoint& wp) {
    switch (wp.kind) {
        case GlobalWaypoint::Gps:
            return QString("%1. 📡 GPS (%2, %3)").arg(i + 1)
                .arg(wp.lon, 0, 'f', 6).arg(wp.lat, 0, 'f', 6);
        case GlobalWaypoint::Instruction:
            return QString("%1. 💬 %2").arg(i + 1).arg(wp.instruction);
        case GlobalWaypoint::Map:
        default:
            return QString("%1. 📍 %2/%3/%4").arg(i + 1)
                .arg(wp.slamMap, wp.submap, wp.semanticPoint);
    }
}

ScenePanel::ScenePanel(QWidget* parent) : QWidget(parent) {
    setupUI();
}

void ScenePanel::setupUI() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    auto* toolbar = new QHBoxLayout();
    auto* btnOsm = new QPushButton("导入底图");
    auto* btnSlam = new QPushButton("导入SLAM地图");
    auto* btnRoute = new QPushButton("+ 全局路线");
    toolbar->addWidget(btnOsm);
    toolbar->addWidget(btnSlam);
    toolbar->addWidget(btnRoute);
    toolbar->addStretch();
    layout->addLayout(toolbar);
    connect(btnOsm,  &QPushButton::clicked, this, &ScenePanel::importOsmRequested);
    connect(btnSlam, &QPushButton::clicked, this, &ScenePanel::addSlamMapRequested);
    connect(btnRoute, &QPushButton::clicked, this, &ScenePanel::addGlobalRouteRequested);

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

    auto* hint = new QLabel("双击 SLAM 地图进入其内部 submap 编辑；勾选控制其正交投影叠加显示。"
                            "全局路线（跨地图）在此管理；内部路线在「SLAM地图」Tab 中随激活地图显示。");
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

    // ── 全局路线（跨地图，展示在场景地图下面）──────────────────────────
    auto* gRoot = new QTreeWidgetItem(tree_, {"🌐 全局路线 (跨地图)"});
    gRoot->setData(0, RoleType, TGlobalRoot);
    gRoot->setExpanded(true);
    gRoot->setFlags(gRoot->flags() & ~Qt::ItemIsSelectable);

    if (scene_) {
        for (const auto& r : scene_->globalRoutes()) {
            auto* rItem = new QTreeWidgetItem({r.name});
            rItem->setData(0, RoleType, TGlobalRoute);
            rItem->setData(0, RoleName, r.name);
            rItem->setToolTip(0, r.description);
            rItem->setCheckState(1, Qt::Unchecked);
            gRoot->addChild(rItem);
            for (int i = 0; i < r.waypoints.size(); ++i) {
                auto* wp = new QTreeWidgetItem({sceneGlobalWpLabel(i, r.waypoints[i])});
                wp->setData(0, RoleType, TWaypoint);
                wp->setData(0, RoleName, r.name);
                wp->setData(0, RoleIndex, i);
                rItem->addChild(wp);
            }
        }
    }

    tree_->blockSignals(false);
}

void ScenePanel::onItemClicked(QTreeWidgetItem* item, int) {
    if (!item) return;
    int type = item->data(0, RoleType).toInt();
    if (type == TSlam)
        emit slamMapSelected(item->data(0, RoleId).toString());
    else if (type == TOsm)
        emit osmSelected();
    else if (type == TGlobalRoute)
        emit globalRouteSelected(item->data(0, RoleName).toString());
}

void ScenePanel::onItemDoubleClicked(QTreeWidgetItem* item, int) {
    if (!item) return;
    if (item->data(0, RoleType).toInt() == TSlam)
        emit slamMapActivated(item->data(0, RoleId).toString());
}

void ScenePanel::onItemChanged(QTreeWidgetItem* item, int column) {
    if (column != 1 || !item) return;
    int type = item->data(0, RoleType).toInt();
    bool checked = (item->checkState(1) == Qt::Checked);
    if (type == TSlam)
        emit orthoVisibilityChanged(item->data(0, RoleId).toString(), checked);
    else if (type == TGlobalRoute)
        emit globalRouteVisibilityChanged(item->data(0, RoleName).toString(), checked);
}

void ScenePanel::onContextMenu(const QPoint& pos) {
    auto* item = tree_->itemAt(pos);
    QMenu menu;
    int type = item ? item->data(0, RoleType).toInt() : -1;
    switch (type) {
    case TSlam: {
        QString id = item->data(0, RoleId).toString();
        menu.addAction("✅ 激活并编辑此地图", [this, id]() { emit slamMapActivated(id); });
        menu.addSeparator();
        menu.addAction("✏️ 重命名此地图", [this, id]() {
            QString cur = id;
            if (scene_) if (auto* e = scene_->mapById(id))
                cur = e->name.isEmpty() ? e->id : e->name;
            bool ok; QString nn = QInputDialog::getText(this, "重命名 SLAM 地图",
                QString("将「%1」重命名为:").arg(cur), QLineEdit::Normal, cur, &ok);
            nn = nn.trimmed();
            if (ok && !nn.isEmpty() && nn != cur) emit renameSlamMapRequested(id, nn);
        });
        menu.addAction("📤 单独导出此地图...", [this, id]() { emit exportSlamMapRequested(id); });
        menu.addSeparator();
        menu.addAction("🏢 导入 SLAM 地图...", [this]() { emit addSlamMapRequested(); });
        menu.addAction("🗑️ 移除此地图", [this, id]() { emit removeSlamMapRequested(id); });
        break;
    }
    case TGlobalRoot:
        menu.addAction("🌐 添加全局路线", [this]() { emit addGlobalRouteRequested(); });
        break;
    case TGlobalRoute: {
        QString name = item->data(0, RoleName).toString();
        menu.addAction("➕ 添加途经点...", [this, name]() { emit addGlobalWaypointRequested(name); });
        menu.addAction("✏️ 重命名", [this, name]() {
            bool ok; QString nn = QInputDialog::getText(this, "重命名全局路线",
                QString("将「%1」重命名为:").arg(name), QLineEdit::Normal, name, &ok);
            if (ok && !nn.isEmpty() && nn != name) emit renameGlobalRouteRequested(name, nn);
        });
        menu.addAction("🗑️ 删除此全局路线", [this, name]() { emit removeGlobalRouteRequested(name); });
        break;
    }
    case TWaypoint: {
        QString name = item->data(0, RoleName).toString();
        if (item->data(0, RoleIndex).isValid() && !name.isEmpty()) {
            int idx = item->data(0, RoleIndex).toInt();
            menu.addAction("🗑️ 删除此途经点", [this, name, idx]() {
                emit removeGlobalWaypointRequested(name, idx);
            });
        }
        break;
    }
    default:
        menu.addAction("🗺️ 导入 OSM/PBF 底图...", [this]() { emit importOsmRequested(); });
        menu.addAction("🏢 导入 SLAM 地图...", [this]() { emit addSlamMapRequested(); });
        menu.addAction("🌐 添加全局路线", [this]() { emit addGlobalRouteRequested(); });
        break;
    }
    menu.exec(tree_->viewport()->mapToGlobal(pos));
}

} // namespace me
