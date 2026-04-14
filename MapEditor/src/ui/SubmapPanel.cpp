#include "ui/SubmapPanel.h"

#include <QVBoxLayout>
#include <QHeaderView>
#include <QMenu>
#include <QMessageBox>
#include <QInputDialog>
#include <QDataStream>
#include <QCheckBox>

namespace me {

// ═══════════════════════════════════════════════════════════════════════
//  DragDropTree — 语义点拖入路线的实现
// ═══════════════════════════════════════════════════════════════════════

static const char* kMimeType      = "application/x-mapeditor-sempoint";
static const char* kRouteMimeType = "application/x-mapeditor-route";

QStringList DragDropTree::mimeTypes() const {
    return {kMimeType, kRouteMimeType};
}

#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
QMimeData* DragDropTree::mimeData(const QList<QTreeWidgetItem*>& items) const {
#else
QMimeData* DragDropTree::mimeData(const QList<QTreeWidgetItem*> items) const {
#endif
    if (items.isEmpty()) return nullptr;
    auto* item = items.first();
    int type = item->data(0, SubmapPanel::RoleType).toInt();

    if (type == SubmapPanel::TypeSemPoint) {
        auto* md = new QMimeData();
        QByteArray buf;
        QDataStream ds(&buf, QIODevice::WriteOnly);
        ds << item->data(0, SubmapPanel::RoleSubmapId).toString()
           << item->data(0, SubmapPanel::RolePointName).toString();
        md->setData(kMimeType, buf);
        return md;
    }

    if (type == SubmapPanel::TypeRoute) {
        auto* md = new QMimeData();
        QByteArray buf;
        QDataStream ds(&buf, QIODevice::WriteOnly);
        ds << item->data(0, SubmapPanel::RoleRouteName).toString();
        md->setData(kRouteMimeType, buf);
        return md;
    }

    return nullptr;
}

Qt::DropActions DragDropTree::supportedDropActions() const {
    return Qt::CopyAction;
}

bool DragDropTree::dropMimeData(QTreeWidgetItem* parent, int index,
                                 const QMimeData* data, Qt::DropAction /*action*/) {
    if (!data || !parent) return false;

    int parentType = parent->data(0, SubmapPanel::RoleType).toInt();

    QString routeName;
    int insertIdx = index;

    if (parentType == SubmapPanel::TypeRoute) {
        routeName = parent->data(0, SubmapPanel::RoleRouteName).toString();
    } else if (parentType == SubmapPanel::TypeRouteWaypoint) {
        auto* routeItem = parent->parent();
        if (!routeItem) return false;
        routeName = routeItem->data(0, SubmapPanel::RoleRouteName).toString();
        insertIdx = routeItem->indexOfChild(parent);
    } else {
        return false;
    }

    if (data->hasFormat(kMimeType)) {
        QByteArray buf = data->data(kMimeType);
        QDataStream ds(&buf, QIODevice::ReadOnly);
        QString submapId, pointName;
        ds >> submapId >> pointName;
        emit waypointDropped(routeName, submapId, pointName, insertIdx);
        return true;
    }

    if (data->hasFormat(kRouteMimeType)) {
        QByteArray buf = data->data(kRouteMimeType);
        QDataStream ds(&buf, QIODevice::ReadOnly);
        QString nestedRouteName;
        ds >> nestedRouteName;
        if (nestedRouteName == routeName) return false;   // 不能自引用
        emit routeRefDropped(routeName, nestedRouteName, insertIdx);
        return true;
    }

    return false;
}

// ═══════════════════════════════════════════════════════════════════════

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

    // 树形列表 (支持拖拽)
    tree_ = new DragDropTree();
    tree_->setHeaderLabels({"名称", "显示"});
    tree_->setColumnWidth(0, 180);
    tree_->setColumnWidth(1, 40);
    tree_->header()->setStretchLastSection(false);
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    tree_->setDragEnabled(true);
    tree_->setAcceptDrops(true);
    tree_->setDropIndicatorShown(true);
    tree_->setDragDropMode(QAbstractItemView::DragDrop);
    tree_->setDefaultDropAction(Qt::CopyAction);

    connect(tree_, &QTreeWidget::itemClicked, this, &SubmapPanel::onItemClicked);
    connect(tree_, &QTreeWidget::itemChanged, this, &SubmapPanel::onItemChanged);
    connect(tree_, &QTreeWidget::customContextMenuRequested, this, &SubmapPanel::onContextMenu);
    connect(tree_, &DragDropTree::waypointDropped, this, &SubmapPanel::routeWaypointInserted);
    connect(tree_, &DragDropTree::routeRefDropped, this, &SubmapPanel::routeRefInserted);

    layout->addWidget(tree_);

    orbitSemanticCenterCk_ = new QCheckBox(QStringLiteral("旋转/缩放以选中语义点为中心"));
    orbitSemanticCenterCk_->setToolTip(
        QStringLiteral("勾选后，在树中点击语义点位时，将以该点在世界坐标系下的位置作为视角旋转与滚轮缩放的中心（不再固定为原点）。\n"
                       "也可在语义点上右键选择「设为视角中心」。"));
    layout->addWidget(orbitSemanticCenterCk_);
}

bool SubmapPanel::orbitCenterFollowsSemantic() const {
    return orbitSemanticCenterCk_ && orbitSemanticCenterCk_->isChecked();
}

// ═══════════════════════════════════════════════════════════════════════
//  从 MapProject 重建树
// ═══════════════════════════════════════════════════════════════════════

void SubmapPanel::rebuild(MapProject* project) {
    project_ = project;

    // 保存展开状态（包括 submap 下的语义点文件夹）
    QSet<QString> expandedRoutes;
    QSet<QString> expandedSubmaps;
    QSet<QString> expandedSemanticFolders;
    for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
        auto* root = tree_->topLevelItem(i);
        for (int j = 0; j < root->childCount(); ++j) {
            auto* child = root->child(j);
            if (!child->isExpanded()) continue;
            int type = child->data(0, RoleType).toInt();
            if (type == TypeRoute) {
                expandedRoutes.insert(child->data(0, RoleRouteName).toString());
            } else if (type == TypeSubmap) {
                QString sid = child->data(0, RoleSubmapId).toString();
                expandedSubmaps.insert(sid);
                // 检查该 submap 的子项中语义点文件夹是否被展开
                for (int k = 0; k < child->childCount(); ++k) {
                    auto* gc = child->child(k);
                    if (!gc) continue;
                    if (gc->data(0, RoleType).toInt() == TypeSemPointsFolder && gc->isExpanded()) {
                        expandedSemanticFolders.insert(sid);
                        break;
                    }
                }
            }
        }
    }

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
        if (expandedSubmaps.contains(id)) smItem->setExpanded(true);
        // 如果之前语义点文件夹是展开的，则展开该文件夹
        if (expandedSemanticFolders.contains(id)) {
            for (int c = 0; c < smItem->childCount(); ++c) {
                auto* sc = smItem->child(c);
                if (sc && sc->data(0, RoleType).toInt() == TypeSemPointsFolder) {
                    sc->setExpanded(true);
                    break;
                }
            }
        }
    }

    // ── Routes ───────────────────────────────────────────────────────
    auto* routeRoot = new QTreeWidgetItem(tree_, {"🛤️ 路线 (Routes)"});
    routeRoot->setData(0, RoleType, TypeRoutesRoot);
    routeRoot->setExpanded(true);
    routeRoot->setFlags(routeRoot->flags() & ~Qt::ItemIsSelectable);

    for (auto it = project_->routes().begin(); it != project_->routes().end(); ++it) {
        auto* rItem = makeRouteItem(it.key(), it.value());
        routeRoot->addChild(rItem);
        if (expandedRoutes.contains(it.key())) rItem->setExpanded(true);
    }

    tree_->blockSignals(false);
}

void SubmapPanel::expandRoute(const QString& routeName) {
    for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
        auto* root = tree_->topLevelItem(i);
        for (int j = 0; j < root->childCount(); ++j) {
            auto* child = root->child(j);
            if (child->data(0, RoleType).toInt() == TypeRoute &&
                child->data(0, RoleRouteName).toString() == routeName) {
                child->setExpanded(true);
                return;
            }
        }
    }
}

QTreeWidgetItem* SubmapPanel::makeSubmapItem(const QString& id, SubMap* sm) {
    QString label = sm->name().isEmpty() ? id : QString("%1 (%2)").arg(sm->name(), id);
    bool isDefault = (project_ && project_->defaultSubmapId() == id);
    if (isDefault) label += " ⭐";
    if (!sm->parentSubmapId().isEmpty())
        label += QString(" → %1").arg(sm->parentSubmapId());

    auto* item = new QTreeWidgetItem({label});
    item->setData(0, RoleType, TypeSubmap);
    item->setData(0, RoleSubmapId, id);
    item->setCheckState(1, sm->displayOptions().visible ? Qt::Checked : Qt::Unchecked);

    // 地图点云（独立控制，但 submap checkbox 为总开关）
    auto* mapCloudItem = new QTreeWidgetItem({"🗺️ 地图点云"});
    mapCloudItem->setData(0, RoleType, TypeMapCloud);
    mapCloudItem->setData(0, RoleSubmapId, id);
    mapCloudItem->setCheckState(1, sm->displayOptions().visible ? Qt::Checked : Qt::Unchecked);
    item->addChild(mapCloudItem);

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
        spItem->setFlags(spItem->flags() | Qt::ItemIsDragEnabled);
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
    bool isDefault = (project_ && project_->defaultRouteId() == name);
    QString label = isDefault ? (name + " ⭐") : name;
    auto* item = new QTreeWidgetItem({label});
    item->setData(0, RoleType, TypeRoute);
    item->setData(0, RoleRouteName, name);
    item->setCheckState(1, Qt::Unchecked);
    item->setToolTip(0, route.description);
    item->setFlags(item->flags() | Qt::ItemIsDropEnabled | Qt::ItemIsDragEnabled);

    for (int i = 0; i < route.waypoints.size(); ++i) {
        const auto& wp = route.waypoints[i];
        QString wpLabel;
        if (wp.isRouteRef()) {
            wpLabel = QString("%1. 🛤️ [%2]").arg(i + 1).arg(wp.routeRef);
        } else {
            wpLabel = QString("%1. %2 / %3").arg(i + 1).arg(wp.submapId, wp.semanticPoint);
        }
        auto* wpItem = new QTreeWidgetItem({wpLabel});
        wpItem->setData(0, RoleType, TypeRouteWaypoint);
        wpItem->setFlags(wpItem->flags() & ~Qt::ItemIsDropEnabled);
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

    // 如果是子项的变化，更新父项（submap）复选框为 全选/部分选/不选
    if (type == TypeMapCloud || type == TypeSemPointsFolder || type == TypePortalsFolder) {
        auto* parent = item->parent();
        if (parent && parent->data(0, RoleType).toInt() == TypeSubmap) {
            int total = 0;
            int checked = 0;
            for (int i = 0; i < parent->childCount(); ++i) {
                auto* c = parent->child(i);
                if (!c) continue;
                int ct = c->data(0, RoleType).toInt();
                if (ct == TypeMapCloud || ct == TypeSemPointsFolder || ct == TypePortalsFolder) {
                    total++;
                    if (c->checkState(1) == Qt::Checked) checked++;
                }
            }
            Qt::CheckState state = Qt::PartiallyChecked;
            if (checked == 0) state = Qt::Unchecked;
            else if (checked == total) state = Qt::Checked;

            tree_->blockSignals(true);
            parent->setCheckState(1, state);
            tree_->blockSignals(false);
        }
    }
}

void SubmapPanel::onItemChanged(QTreeWidgetItem* item, int column) {
    if (column != 1 || !item) return;

    int type = item->data(0, RoleType).toInt();
    QString smId = item->data(0, RoleSubmapId).toString();
    bool checked = (item->checkState(1) == Qt::Checked);

    switch (type) {
    case TypeSubmap: {
        // 主开关：同时控制地图点云、语义点位和传送门
        // 先更新子项的复选框（防止递归 signal）
        tree_->blockSignals(true);
        for (int i = 0; i < item->childCount(); ++i) {
            auto* c = item->child(i);
            if (!c) continue;
            int ct = c->data(0, RoleType).toInt();
            if (ct == TypeMapCloud || ct == TypeSemPointsFolder || ct == TypePortalsFolder) {
                c->setCheckState(1, checked ? Qt::Checked : Qt::Unchecked);
            }
        }
        tree_->blockSignals(false);

        emit submapVisibilityChanged(smId, checked);
        emit showSemanticPointsChanged(smId, checked);
        emit showPortalsChanged(smId, checked);
        break;
    }
    case TypeMapCloud:
        emit submapVisibilityChanged(smId, checked);
        // fallthrough to update parent master checkbox below
        break;
    case TypeSemPointsFolder:
        emit showSemanticPointsChanged(smId, checked);
        break;
    case TypePortalsFolder:
        emit showPortalsChanged(smId, checked);
        break;
    case TypeRoute:
        emit routeVisibilityChanged(item->data(0, RoleRouteName).toString(), checked);
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
        bool isDefault = (project_ && project_->defaultSubmapId() == smId);
        if (!isDefault) {
            menu.addAction("⭐ 设为默认地图", [this, smId]() {
                emit setDefaultSubmapRequested(smId);
            });
            menu.addSeparator();
        }

        // 设置父地图
        {
            auto* parentMenu = menu.addMenu("🔗 设置父地图");
            auto* sm = project_ ? project_->submap(smId) : nullptr;
            QString currentParent = sm ? sm->parentSubmapId() : QString();

            if (!currentParent.isEmpty()) {
                parentMenu->addAction(QString("✖ 清除父节点 (当前: %1)").arg(currentParent),
                    [this, smId]() { emit setParentSubmapRequested(smId, QString()); });
                parentMenu->addSeparator();
            }
            if (project_) {
                for (const auto& otherId : project_->submapIds()) {
                    if (otherId == smId || otherId == currentParent) continue;
                    parentMenu->addAction(QString("→ %1").arg(otherId),
                        [this, smId, otherId]() { emit setParentSubmapRequested(smId, otherId); });
                }
            }
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
        menu.addAction("🌳 显示 TF 树", [this]() {
            emit showTfTreeRequested();
        });
        if (!isDefault) {
            menu.addSeparator();
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
        bool isDefaultRoute = (project_ && project_->defaultRouteId() == rName);
        if (!isDefaultRoute) {
            menu.addAction("⭐ 设为默认路线", [this, rName]() {
                emit setDefaultRouteRequested(rName);
            });
            menu.addSeparator();
        }
        menu.addAction("✏️ 重命名路线", [this, rName]() {
            bool ok;
            QString newName = QInputDialog::getText(
                this, "重命名路线",
                QString("将路线「%1」重命名为:").arg(rName),
                QLineEdit::Normal, rName, &ok);
            if (ok && !newName.isEmpty() && newName != rName)
                emit renameRouteRequested(rName, newName);
        });
        menu.addAction("🗑️ 删除此路线", [this, rName]() {
            emit removeRouteRequested(rName);
        });
        break;
    }
    case TypeRouteWaypoint: {
        auto* routeItem = item->parent();
        if (!routeItem || routeItem->data(0, RoleType).toInt() != TypeRoute) {
            return;
        }
        QString rName = routeItem->data(0, RoleRouteName).toString();
        int wpIdx = routeItem->indexOfChild(item);
        if (wpIdx < 0) return;
        menu.addAction("🗑️ 删除此途经点", [this, rName, wpIdx]() {
            emit routeWaypointRemoveRequested(rName, wpIdx);
        });
        break;
    }
    case TypeSemPoint: {
        QString ptName = item->data(0, RolePointName).toString();
        menu.addAction("🎯 设为视角旋转/缩放中心", [this, smId, ptName]() {
            emit orbitCenterAtSemanticRequested(smId, ptName);
        });
        break;
    }
    default:
        return;
    }

    menu.exec(tree_->viewport()->mapToGlobal(pos));
}

} // namespace me
