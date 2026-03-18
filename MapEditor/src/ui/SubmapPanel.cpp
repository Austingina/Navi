#include "ui/SubmapPanel.h"

#include <QVBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QMessageBox>

namespace me {

SubmapPanel::SubmapPanel(QWidget* parent) : QWidget(parent) {
    setupUI();
}

void SubmapPanel::setupUI() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 工具栏
    auto* toolbar = new QHBoxLayout();
    btnAdd_ = new QPushButton("+ 新增 Submap");
    toolbar->addWidget(btnAdd_);
    toolbar->addStretch();
    layout->addLayout(toolbar);

    connect(btnAdd_, &QPushButton::clicked, this, &SubmapPanel::addSubmapRequested);

    // 树形列表
    tree_ = new QTreeWidget();
    tree_->setHeaderLabels({"名称", "显示"});
    tree_->setColumnWidth(0, 180);
    tree_->setColumnWidth(1, 40);
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    tree_->setSelectionMode(QAbstractItemView::SingleSelection);

    connect(tree_, &QTreeWidget::itemClicked, this, &SubmapPanel::onItemClicked);
    connect(tree_, &QTreeWidget::itemChanged, this, &SubmapPanel::onItemChanged);
    connect(tree_, &QTreeWidget::customContextMenuRequested, this, &SubmapPanel::onContextMenu);

    layout->addWidget(tree_);
}

// ═══════════════════════════════════════════════════════════════════════
//  从 MapProject 重建树
// ═══════════════════════════════════════════════════════════════════════

void SubmapPanel::rebuild(MapProject* project) {
    project_ = project;
    tree_->blockSignals(true);
    tree_->clear();

    if (!project_) {
        tree_->blockSignals(false);
        return;
    }

    // ── Submaps ──────────────────────────────────────────────────────
    auto* submapRoot = new QTreeWidgetItem(tree_, {"📦 子地图 (Submaps)"});
    submapRoot->setData(0, RoleType, TypeSubmapRoot);
    submapRoot->setExpanded(true);
    submapRoot->setFlags(submapRoot->flags() & ~Qt::ItemIsSelectable);

    for (const auto& id : project_->submapIds()) {
        auto* sm = project_->submap(id);
        if (!sm) continue;
        auto* smItem = makeSubmapItem(id, sm);
        submapRoot->addChild(smItem);
    }

    // ── Routes ───────────────────────────────────────────────────────
    auto* routeRoot = new QTreeWidgetItem(tree_, {"🛤️ 路线 (Routes)"});
    routeRoot->setData(0, RoleType, TypeRoutesRoot);
    routeRoot->setExpanded(true);
    routeRoot->setFlags(routeRoot->flags() & ~Qt::ItemIsSelectable);

    for (auto it = project_->routes().begin(); it != project_->routes().end(); ++it) {
        auto* rItem = makeRouteItem(it.key(), it.value());
        routeRoot->addChild(rItem);
    }

    tree_->blockSignals(false);
}

QTreeWidgetItem* SubmapPanel::makeSubmapItem(const QString& id, SubMap* sm) {
    QString label = sm->name().isEmpty() ? id : QString("%1 (%2)").arg(sm->name(), id);
    bool isDefault = (project_ && project_->defaultSubmapId() == id);
    if (isDefault) label += " ⭐";

    auto* item = new QTreeWidgetItem({label});
    item->setData(0, RoleType, TypeSubmap);
    item->setData(0, RoleSubmapId, id);
    item->setCheckState(1, sm->displayOptions().visible ? Qt::Checked : Qt::Unchecked);

    // 语义点位文件夹
    auto* spFolder = new QTreeWidgetItem({"📍 语义点位"});
    spFolder->setData(0, RoleType, TypeSemPointsFolder);
    spFolder->setData(0, RoleSubmapId, id);
    spFolder->setCheckState(1, sm->displayOptions().showSemanticPts ? Qt::Checked : Qt::Unchecked);

    for (auto sit = sm->semanticPoints().begin(); sit != sm->semanticPoints().end(); ++sit) {
        auto* spItem = new QTreeWidgetItem({sit.key()});
        spItem->setData(0, RoleType, TypeSemPoint);
        spItem->setData(0, RoleSubmapId, id);
        spItem->setData(0, RolePointName, sit.key());
        spFolder->addChild(spItem);
    }
    item->addChild(spFolder);

    // 传送门文件夹
    auto* ptFolder = new QTreeWidgetItem({"🚪 传送门"});
    ptFolder->setData(0, RoleType, TypePortalsFolder);
    ptFolder->setData(0, RoleSubmapId, id);
    ptFolder->setCheckState(1, sm->displayOptions().showPortals ? Qt::Checked : Qt::Unchecked);

    for (int i = 0; i < sm->portals().size(); ++i) {
        const auto& p = sm->portals()[i];
        auto* pItem = new QTreeWidgetItem({QString("%1 → %2").arg(p.portalName, p.targetSubmap)});
        pItem->setData(0, RoleType, TypePortal);
        pItem->setData(0, RoleSubmapId, id);
        pItem->setData(0, RolePortalIdx, i);
        ptFolder->addChild(pItem);
    }
    item->addChild(ptFolder);

    return item;
}

QTreeWidgetItem* SubmapPanel::makeRouteItem(const QString& name, const Route& route) {
    auto* item = new QTreeWidgetItem({name});
    item->setData(0, RoleType, TypeRoute);
    item->setData(0, RoleRouteName, name);
    item->setToolTip(0, route.description);

    for (int i = 0; i < route.waypoints.size(); ++i) {
        const auto& wp = route.waypoints[i];
        auto* wpItem = new QTreeWidgetItem({
            QString("%1. %2 / %3").arg(i + 1).arg(wp.submapId, wp.semanticPoint)});
        wpItem->setData(0, RoleType, TypeRouteWaypoint);
        item->addChild(wpItem);
    }
    return item;
}

// ═══════════════════════════════════════════════════════════════════════
//  事件处理
// ═══════════════════════════════════════════════════════════════════════

void SubmapPanel::onItemClicked(QTreeWidgetItem* item, int /*column*/) {
    if (!item) return;
    int type = item->data(0, RoleType).toInt();
    QString smId = item->data(0, RoleSubmapId).toString();

    switch (type) {
    case TypeSubmap:
        emit submapSelected(smId);
        break;
    case TypeSemPoint:
        emit semanticPointSelected(smId, item->data(0, RolePointName).toString());
        break;
    case TypePortal:
        emit portalSelected(smId, item->data(0, RolePortalIdx).toInt());
        break;
    case TypeRoute:
        emit routeSelected(item->data(0, RoleRouteName).toString());
        break;
    }
}

void SubmapPanel::onItemChanged(QTreeWidgetItem* item, int column) {
    if (column != 1 || !item) return;

    int type = item->data(0, RoleType).toInt();
    QString smId = item->data(0, RoleSubmapId).toString();
    bool checked = (item->checkState(1) == Qt::Checked);

    switch (type) {
    case TypeSubmap:
        emit submapVisibilityChanged(smId, checked);
        break;
    case TypeSemPointsFolder:
        emit showSemanticPointsChanged(smId, checked);
        break;
    case TypePortalsFolder:
        emit showPortalsChanged(smId, checked);
        break;
    }
}

void SubmapPanel::onContextMenu(const QPoint& pos) {
    auto* item = tree_->itemAt(pos);
    if (!item) return;

    int type = item->data(0, RoleType).toInt();
    QString smId = item->data(0, RoleSubmapId).toString();

    QMenu menu;

    switch (type) {
    case TypeSubmap: {
        // 设置为 default_submap
        bool isDefault = (project_ && project_->defaultSubmapId() == smId);
        if (!isDefault) {
            menu.addAction("⭐ 设为默认子地图", [this, smId]() {
                emit setDefaultSubmapRequested(smId);
            });
            menu.addSeparator();
        }
        menu.addAction("🔧 对齐 Transform (3D-BBS)", [this, smId]() {
            emit alignSubmapRequested(smId);
        });
        menu.addAction("📍 添加语义点位", [this, smId]() {
            emit addSemanticPointRequested(smId);
        });
        menu.addAction("🚪 添加传送门", [this, smId]() {
            emit addPortalRequested(smId);
        });
        menu.addSeparator();
        if (!isDefault) {
            menu.addAction("🗑️ 删除此 Submap", [this, smId]() {
                emit removeSubmapRequested(smId);
            });
        }
        break;
    }
    case TypeSemPointsFolder:
        menu.addAction("📍 添加语义点位", [this, smId]() {
            emit addSemanticPointRequested(smId);
        });
        break;
    case TypePortalsFolder:
        menu.addAction("🚪 添加传送门", [this, smId]() {
            emit addPortalRequested(smId);
        });
        break;
    case TypeRoutesRoot:
        menu.addAction("🛤️ 添加路线", [this]() {
            emit addRouteRequested();
        });
        break;
    case TypeRoute: {
        QString rName = item->data(0, RoleRouteName).toString();
        menu.addAction("🗑️ 删除此路线", [this, rName]() {
            emit removeRouteRequested(rName);
        });
        break;
    }
    default:
        return;
    }

    menu.exec(tree_->viewport()->mapToGlobal(pos));
}

} // namespace me
