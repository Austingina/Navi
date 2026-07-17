#include "ui/RoutePanel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QMenu>
#include <QInputDialog>
#include <QLineEdit>

namespace me {

RoutePanel::RoutePanel(QWidget* parent) : QWidget(parent) {
    setupUI();
}

void RoutePanel::setupUI() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    auto* toolbar = new QHBoxLayout();
    btnAddGlobal_ = new QPushButton("+ 全局路线");
    toolbar->addWidget(btnAddGlobal_);
    toolbar->addStretch();
    layout->addLayout(toolbar);
    connect(btnAddGlobal_, &QPushButton::clicked, this, &RoutePanel::addGlobalRouteRequested);

    tree_ = new QTreeWidget();
    tree_->setHeaderLabels({"名称", "显示"});
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->header()->setStretchLastSection(false);
    tree_->setColumnWidth(1, 40);
    tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(tree_, &QTreeWidget::customContextMenuRequested, this, &RoutePanel::onContextMenu);
    connect(tree_, &QTreeWidget::itemClicked, this, &RoutePanel::onItemClicked);
    connect(tree_, &QTreeWidget::itemChanged, this, &RoutePanel::onItemChanged);
    layout->addWidget(tree_);
}

static QString globalWpLabel(int i, const GlobalWaypoint& wp) {
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

void RoutePanel::rebuild(Scene* scene) {
    scene_ = scene;
    tree_->blockSignals(true);
    tree_->clear();

    // ── 全局路线 ───────────────────────────────────────────────────────
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
                auto* wp = new QTreeWidgetItem({globalWpLabel(i, r.waypoints[i])});
                wp->setData(0, RoleType, TWaypoint);
                wp->setData(0, RoleName, r.name);
                wp->setData(0, RoleIndex, i);
                rItem->addChild(wp);
            }
        }
    }

    // ── 内部路线（按地图分组）─────────────────────────────────────────
    auto* iRoot = new QTreeWidgetItem(tree_, {"🏢 内部路线 (各 SLAM 地图)"});
    iRoot->setData(0, RoleType, TInternalRoot);
    iRoot->setExpanded(true);
    iRoot->setFlags(iRoot->flags() & ~Qt::ItemIsSelectable);

    if (scene_) {
        for (const auto& e : scene_->maps()) {
            auto* grp = new QTreeWidgetItem({QString("🏢 %1 (%2)")
                                                 .arg(e.name.isEmpty() ? e.id : e.name, e.id)});
            grp->setData(0, RoleType, TMapGroup);
            grp->setData(0, RoleMapId, e.id);
            grp->setExpanded(true);
            iRoot->addChild(grp);
            if (!e.project) continue;
            for (auto it = e.project->routes().begin(); it != e.project->routes().end(); ++it) {
                auto* rItem = new QTreeWidgetItem({it.key()});
                rItem->setData(0, RoleType, TInternalRoute);
                rItem->setData(0, RoleName, it.key());
                rItem->setData(0, RoleMapId, e.id);
                rItem->setToolTip(0, it.value().description);
                rItem->setCheckState(1, Qt::Unchecked);
                grp->addChild(rItem);
                for (int i = 0; i < it.value().waypoints.size(); ++i) {
                    const auto& wp = it.value().waypoints[i];
                    QString lbl = wp.isRouteRef()
                        ? QString("%1. 🛤️ [%2]").arg(i + 1).arg(wp.routeRef)
                        : QString("%1. %2/%3").arg(i + 1).arg(wp.submapId, wp.semanticPoint);
                    auto* w = new QTreeWidgetItem({lbl});
                    w->setData(0, RoleType, TWaypoint);
                    rItem->addChild(w);
                }
            }
        }
    }

    tree_->blockSignals(false);
}

void RoutePanel::onItemClicked(QTreeWidgetItem* item, int) {
    if (!item) return;
    switch (item->data(0, RoleType).toInt()) {
        case TGlobalRoute:
            emit globalRouteSelected(item->data(0, RoleName).toString());
            break;
        case TInternalRoute:
            emit internalRouteSelected(item->data(0, RoleMapId).toString(),
                                       item->data(0, RoleName).toString());
            break;
    }
}

void RoutePanel::onItemChanged(QTreeWidgetItem* item, int column) {
    if (column != 1 || !item) return;
    bool checked = (item->checkState(1) == Qt::Checked);
    switch (item->data(0, RoleType).toInt()) {
        case TGlobalRoute:
            emit globalRouteVisibilityChanged(item->data(0, RoleName).toString(), checked);
            break;
        case TInternalRoute:
            emit internalRouteVisibilityChanged(item->data(0, RoleMapId).toString(),
                                                item->data(0, RoleName).toString(), checked);
            break;
    }
}

void RoutePanel::onContextMenu(const QPoint& pos) {
    auto* item = tree_->itemAt(pos);
    QMenu menu;

    int type = item ? item->data(0, RoleType).toInt() : -1;
    switch (type) {
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
            // 仅全局路线的 waypoint 支持删除（内部路线 waypoint 走各地图逻辑）
            QString name = item->data(0, RoleName).toString();
            bool hasIndex = item->data(0, RoleIndex).isValid();
            if (hasIndex && !name.isEmpty()) {
                int idx = item->data(0, RoleIndex).toInt();
                menu.addAction("🗑️ 删除此途经点", [this, name, idx]() {
                    emit removeGlobalWaypointRequested(name, idx);
                });
            }
            break;
        }
        case TMapGroup: {
            QString mapId = item->data(0, RoleMapId).toString();
            menu.addAction("➕ 为此地图添加内部路线", [this, mapId]() {
                emit addInternalRouteRequested(mapId);
            });
            break;
        }
        case TInternalRoute: {
            QString mapId = item->data(0, RoleMapId).toString();
            QString name = item->data(0, RoleName).toString();
            menu.addAction("🗑️ 删除此内部路线", [this, mapId, name]() {
                emit removeInternalRouteRequested(mapId, name);
            });
            break;
        }
        default:
            menu.addAction("🌐 添加全局路线", [this]() { emit addGlobalRouteRequested(); });
            break;
    }
    menu.exec(tree_->viewport()->mapToGlobal(pos));
}

} // namespace me
