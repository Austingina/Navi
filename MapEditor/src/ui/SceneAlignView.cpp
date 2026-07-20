#include "ui/SceneAlignView.h"

#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtMath>
#include <limits>
#include <cmath>

namespace me {

SceneAlignView::SceneAlignView(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(400, 300);
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(30, 32, 36));
    setPalette(pal);
}

// ═══════════════════════════════════════════════════════════════════════
//  足迹栅格化：root 系点云 → 俯视 QImage
// ═══════════════════════════════════════════════════════════════════════
bool SceneAlignView::buildFootprint(CloudPtr cloud, const QColor& color, SlamItem& out) {
    if (!cloud || cloud->empty()) return false;

    double minx = std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double maxx = -std::numeric_limits<double>::max();
    double maxy = -std::numeric_limits<double>::max();
    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
        minx = std::min(minx, (double)pt.x);
        miny = std::min(miny, (double)pt.y);
        maxx = std::max(maxx, (double)pt.x);
        maxy = std::max(maxy, (double)pt.y);
    }
    if (maxx <= minx || maxy <= miny) return false;

    const double extent = std::max(maxx - minx, maxy - miny);
    double ppm = 6.0;                        // 目标分辨率
    const double kMaxPx = 1600.0;
    if (extent * ppm > kMaxPx) ppm = kMaxPx / extent;
    if (ppm < 0.05) ppm = 0.05;

    const int W = (int)std::ceil((maxx - minx) * ppm) + 2;
    const int H = (int)std::ceil((maxy - miny) * ppm) + 2;
    if (W <= 0 || H <= 0 || (qint64)W * H > 8'000'000LL) return false;

    QImage img(W, H, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::transparent);

    const QRgb rgb = qPremultiply(qRgba(color.red(), color.green(), color.blue(), 200));
    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
        const int px = (int)((pt.x - minx) * ppm);
        const int py = (int)((maxy - pt.y) * ppm);   // 图像 y 向下
        if (px < 0 || px >= W || py < 0 || py >= H) continue;
        img.setPixel(px, py, rgb);
    }

    out.footprint = img;
    out.localMinX = minx;
    out.localMaxY = maxy;
    out.pxPerM    = ppm;
    out.color     = color;
    out.valid     = true;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  OSM / 原点
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignView::setOsm(std::shared_ptr<OsmData> osm, double originLon, double originLat) {
    osm_ = std::move(osm);
    conv_.setOrigin(originLon, originLat);
    update();
}

void SceneAlignView::setOrigin(double originLon, double originLat) {
    conv_.setOrigin(originLon, originLat);
    update();
}

// ═══════════════════════════════════════════════════════════════════════
//  SLAM 条目管理
// ═══════════════════════════════════════════════════════════════════════
int SceneAlignView::addSlamItem(const QString& id, const QString& name,
                                CloudPtr rootCloud, const QColor& color,
                                const GeoAnchor& anchor) {
    SlamItem it;
    it.id   = id;
    it.name = name;
    it.anchor = anchor;
    if (!buildFootprint(rootCloud, color, it)) {
        it.valid = false;
        it.color = color;
    }
    items_.append(it);
    selected_ = items_.size() - 1;
    update();
    return selected_;
}

void SceneAlignView::updateAnchor(int index, const GeoAnchor& anchor) {
    if (index < 0 || index >= items_.size()) return;
    items_[index].anchor = anchor;
    update();
}

GeoAnchor SceneAlignView::anchor(int index) const {
    if (index < 0 || index >= items_.size()) return {};
    return items_[index].anchor;
}

void SceneAlignView::removeItem(int index) {
    if (index < 0 || index >= items_.size()) return;
    items_.remove(index);
    if (selected_ >= items_.size()) selected_ = items_.size() - 1;
    update();
}

void SceneAlignView::clearItems() {
    items_.clear();
    selected_ = -1;
    update();
}

int SceneAlignView::indexOfItem(const QString& id) const {
    for (int i = 0; i < items_.size(); ++i)
        if (items_[i].id == id) return i;
    return -1;
}

void SceneAlignView::setItemVisible(const QString& id, bool visible) {
    int i = indexOfItem(id);
    if (i < 0) return;
    items_[i].visible = visible;
    update();
}

void SceneAlignView::setSelected(int index) {
    if (index < -1 || index >= items_.size()) return;
    selected_ = index;
    update();
}

// ═══════════════════════════════════════════════════════════════════════
//  场景语义点位
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignView::setScenePoints(const QVector<SceneSemanticPoint>& pts) {
    scenePoints_ = pts;
    update();
}

void SceneAlignView::setSelectedScenePoint(const QString& name) {
    selectedPoint_ = name;
    update();
}

void SceneAlignView::setAddPointMode(bool on) {
    addPointMode_ = on;
    setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
    update();
}

int SceneAlignView::hitTestPoint(const QPointF& screenPos) const {
    if (!conv_.hasOrigin()) return -1;
    const double kHitPx = 10.0;
    int best = -1;
    double bestD = kHitPx;
    for (int i = 0; i < scenePoints_.size(); ++i) {
        double e, n; conv_.toEnu(scenePoints_[i].lon, scenePoints_[i].lat, e, n);
        QPointF s = enuToScreen(e, n);
        double d = std::hypot(s.x() - screenPos.x(), s.y() - screenPos.y());
        if (d <= bestD) { bestD = d; best = i; }
    }
    return best;
}

// ═══════════════════════════════════════════════════════════════════════
//  坐标换算
// ═══════════════════════════════════════════════════════════════════════
QPointF SceneAlignView::enuToScreen(double e, double n) const {
    return QPointF((e - viewCenterE_) * viewScale_ + width() * 0.5,
                   -(n - viewCenterN_) * viewScale_ + height() * 0.5);
}

void SceneAlignView::screenToEnu(const QPointF& p, double& e, double& n) const {
    e = (p.x() - width() * 0.5) / viewScale_ + viewCenterE_;
    n = -(p.y() - height() * 0.5) / viewScale_ + viewCenterN_;
}

void SceneAlignView::itemOriginEnu(const SlamItem& it, double& e, double& n) const {
    conv_.toEnu(it.anchor.lon, it.anchor.lat, e, n);
}

// 组合变换：足迹像素 → 屏幕
QTransform SceneAlignView::itemPixelToScreen(const SlamItem& it) const {
    const double ppm = it.pxPerM > 0 ? it.pxPerM : 1.0;
    // P: 像素 → 局部米
    QTransform P(1.0 / ppm, 0.0, 0.0, -1.0 / ppm, it.localMinX, it.localMaxY);

    // L: 局部米 → ENU (缩放 k + 朝向 h + 平移到 origin)
    const double k = it.anchor.scale != 0.0 ? it.anchor.scale : 1.0;
    const double h = qDegreesToRadians(it.anchor.headingDeg);
    double e0, n0;
    itemOriginEnu(it, e0, n0);
    const double c = std::cos(h) * k;
    const double s = std::sin(h) * k;
    QTransform L(c, s, -s, c, e0, n0);

    // V: ENU → 屏幕
    const double sc = viewScale_;
    QTransform V(sc, 0.0, 0.0, -sc,
                 -sc * viewCenterE_ + width() * 0.5,
                 sc * viewCenterN_ + height() * 0.5);

    // 行向量约定：point * (P * L * V) 依次应用 P、L、V
    return P * L * V;
}

QPolygonF SceneAlignView::itemScreenPolygon(const SlamItem& it) const {
    QTransform t = itemPixelToScreen(it);
    const int W = it.footprint.width();
    const int H = it.footprint.height();
    QPolygonF poly;
    poly << t.map(QPointF(0, 0))
         << t.map(QPointF(W, 0))
         << t.map(QPointF(W, H))
         << t.map(QPointF(0, H));
    return poly;
}

int SceneAlignView::hitTest(const QPointF& screenPos) const {
    // 从上层(后绘制)向下测试
    for (int i = items_.size() - 1; i >= 0; --i) {
        if (!items_[i].valid || !items_[i].visible) continue;
        if (itemScreenPolygon(items_[i]).containsPoint(screenPos, Qt::OddEvenFill))
            return i;
    }
    return -1;
}

// ═══════════════════════════════════════════════════════════════════════
//  fitView：把 OSM + 所有足迹纳入视野
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignView::fitView() {
    double minE = std::numeric_limits<double>::max();
    double minN = std::numeric_limits<double>::max();
    double maxE = -std::numeric_limits<double>::max();
    double maxN = -std::numeric_limits<double>::max();
    bool any = false;

    auto acc = [&](double e, double n) {
        minE = std::min(minE, e); maxE = std::max(maxE, e);
        minN = std::min(minN, n); maxN = std::max(maxN, n);
        any = true;
    };

    if (osm_ && conv_.hasOrigin()) {
        for (const auto& nd : osm_->nodes()) {
            double e, n; conv_.toEnu(nd.lon, nd.lat, e, n); acc(e, n);
        }
    }
    for (const auto& it : items_) {
        if (!it.valid) continue;
        QTransform t = itemPixelToScreen(it);
        // 逆用：直接用局部→ENU 的角点
        const double ppm = it.pxPerM > 0 ? it.pxPerM : 1.0;
        const double lx0 = it.localMinX, ly1 = it.localMaxY;
        const double lx1 = it.localMinX + it.footprint.width() / ppm;
        const double ly0 = it.localMaxY - it.footprint.height() / ppm;
        const double k = it.anchor.scale != 0.0 ? it.anchor.scale : 1.0;
        const double hh = qDegreesToRadians(it.anchor.headingDeg);
        double e0, n0; itemOriginEnu(it, e0, n0);
        const double c = std::cos(hh) * k, s = std::sin(hh) * k;
        auto local2enu = [&](double lx, double ly) {
            acc(c * lx - s * ly + e0, s * lx + c * ly + n0);
        };
        local2enu(lx0, ly0); local2enu(lx1, ly0);
        local2enu(lx1, ly1); local2enu(lx0, ly1);
        Q_UNUSED(t);
    }

    if (!any) { viewCenterE_ = viewCenterN_ = 0.0; viewScale_ = 4.0; update(); return; }

    viewCenterE_ = 0.5 * (minE + maxE);
    viewCenterN_ = 0.5 * (minN + maxN);
    const double spanE = std::max(1.0, maxE - minE);
    const double spanN = std::max(1.0, maxN - minN);
    const double sx = width()  / (spanE * 1.15);
    const double sy = height() / (spanN * 1.15);
    viewScale_ = std::max(0.02, std::min(sx, sy));
    update();
}

// ═══════════════════════════════════════════════════════════════════════
//  绘制
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(30, 32, 36));
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);

    drawOsm(p);

    // SLAM 足迹
    for (int i = 0; i < items_.size(); ++i) {
        const auto& it = items_[i];
        if (!it.valid || !it.visible) continue;
        p.save();
        p.setWorldTransform(itemPixelToScreen(it), false);
        p.drawImage(0, 0, it.footprint);
        p.restore();

        // 边框 + 原点 + 名称
        QPolygonF poly = itemScreenPolygon(it);
        const bool sel = (i == selected_);
        QPen pen(sel ? QColor(255, 210, 70) : it.color.lighter(140));
        pen.setWidthF(sel ? 2.4 : 1.2);
        if (!sel) pen.setStyle(Qt::DashLine);
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);
        p.drawPolygon(poly);

        double e0, n0; itemOriginEnu(it, e0, n0);
        QPointF o = enuToScreen(e0, n0);
        p.setBrush(sel ? QColor(255, 210, 70) : it.color);
        p.setPen(QPen(Qt::black, 1));
        p.drawEllipse(o, 4, 4);

        // 朝向指示 (局部 +X)
        const double h = qDegreesToRadians(it.anchor.headingDeg);
        QPointF dir(o.x() + std::cos(h) * 28.0, o.y() - std::sin(h) * 28.0);
        p.setPen(QPen(sel ? QColor(255, 210, 70) : it.color, sel ? 2.4 : 1.4));
        p.drawLine(o, dir);

        p.setPen(sel ? QColor(255, 230, 140) : QColor(220, 220, 220));
        p.drawText(poly.boundingRect().topLeft() + QPointF(4, -4), it.name);
    }

    drawScenePoints(p);
    drawScaleBar(p);

    // 提示
    p.setPen(QColor(170, 175, 185));
    if (addPointMode_)
        p.drawText(rect().adjusted(8, 6, -8, 0), Qt::AlignLeft | Qt::AlignTop,
                   "点选模式：在底图上点击以确定语义点位位置");
    else
        p.drawText(rect().adjusted(8, 6, -8, 0), Qt::AlignLeft | Qt::AlignTop,
                   "左键拖拽:平移  右键/Alt+左键:旋转  Shift+左键:缩放  中键/空白拖拽:移视图  滚轮:视图缩放");
}

void SceneAlignView::drawScenePoints(QPainter& p) const {
    if (!conv_.hasOrigin() || scenePoints_.isEmpty()) return;
    for (const auto& pt : scenePoints_) {
        double e, n; conv_.toEnu(pt.lon, pt.lat, e, n);
        QPointF o = enuToScreen(e, n);
        const bool sel = (!selectedPoint_.isEmpty() && pt.name == selectedPoint_);
        const QColor col = sel ? QColor(255, 210, 70) : QColor(90, 220, 180);

        // 航向指示：与正北的夹角，顺时针为正（北=屏幕上方）
        const double b = qDegreesToRadians(pt.headingDeg);
        QPointF dir(o.x() + std::sin(b) * 24.0, o.y() - std::cos(b) * 24.0);
        p.setPen(QPen(col, sel ? 2.4 : 1.6));
        p.drawLine(o, dir);
        // 箭头
        const double ah = qDegreesToRadians(pt.headingDeg + 155.0);
        const double ah2 = qDegreesToRadians(pt.headingDeg - 155.0);
        p.drawLine(dir, QPointF(dir.x() + std::sin(ah) * 7.0, dir.y() - std::cos(ah) * 7.0));
        p.drawLine(dir, QPointF(dir.x() + std::sin(ah2) * 7.0, dir.y() - std::cos(ah2) * 7.0));

        p.setBrush(col);
        p.setPen(QPen(Qt::black, 1));
        p.drawEllipse(o, sel ? 6 : 5, sel ? 6 : 5);

        p.setPen(sel ? QColor(255, 235, 150) : QColor(210, 235, 225));
        p.drawText(o + QPointF(8, -6), pt.name);
    }
}

void SceneAlignView::drawOsm(QPainter& p) const {
    if (!osm_ || !conv_.hasOrigin()) return;
    const auto& nodes = osm_->nodes();

    auto wayPoly = [&](const OsmWay& w) {
        QPolygonF poly;
        poly.reserve(w.refs.size());
        for (qint64 ref : w.refs) {
            auto it = nodes.constFind(ref);
            if (it == nodes.constEnd()) continue;
            double e, n; conv_.toEnu(it->lon, it->lat, e, n);
            poly << enuToScreen(e, n);
        }
        return poly;
    };

    // 先画面（建筑/水体），再画线
    for (const auto& w : osm_->ways()) {
        if (!w.area) continue;
        QPolygonF poly = wayPoly(w);
        if (poly.size() < 3) continue;
        if (w.kind == OsmWayKind::Building) {
            p.setPen(QPen(QColor(150, 140, 120), 1.0));
            p.setBrush(QColor(90, 82, 70, 180));
        } else if (w.kind == OsmWayKind::Water) {
            p.setPen(QPen(QColor(90, 130, 170), 1.0));
            p.setBrush(QColor(60, 90, 130, 160));
        } else {
            p.setPen(QPen(QColor(90, 95, 105), 1.0));
            p.setBrush(QColor(60, 64, 72, 120));
        }
        p.drawPolygon(poly);
    }

    for (const auto& w : osm_->ways()) {
        if (w.area) continue;
        QPolygonF poly = wayPoly(w);
        if (poly.size() < 2) continue;
        QPen pen;
        switch (w.kind) {
            case OsmWayKind::Highway:
                pen = QPen(QColor(180, 180, 185), 2.4); break;
            case OsmWayKind::Footway:
                pen = QPen(QColor(120, 190, 120), 1.6);
                pen.setStyle(Qt::DashLine); break;
            case OsmWayKind::Water:
                pen = QPen(QColor(90, 140, 190), 1.8); break;
            case OsmWayKind::Barrier:
                pen = QPen(QColor(150, 110, 80), 1.2); break;
            default:
                pen = QPen(QColor(80, 84, 92), 1.0); break;
        }
        p.setBrush(Qt::NoBrush);
        p.setPen(pen);
        p.drawPolyline(poly);
    }

    // 场景原点十字
    QPointF o = enuToScreen(0, 0);
    p.setPen(QPen(QColor(200, 80, 80), 1.2));
    p.drawLine(o + QPointF(-8, 0), o + QPointF(8, 0));
    p.drawLine(o + QPointF(0, -8), o + QPointF(0, 8));
}

void SceneAlignView::drawScaleBar(QPainter& p) const {
    // 选一个"整齐"的米数，使像素长度落在 60~140 之间
    double targetPx = 100.0;
    double meters = targetPx / viewScale_;
    double pow10 = std::pow(10.0, std::floor(std::log10(std::max(1e-6, meters))));
    double norm = meters / pow10;
    double nice = (norm < 1.5) ? 1 : (norm < 3.5) ? 2 : (norm < 7.5) ? 5 : 10;
    meters = nice * pow10;
    double px = meters * viewScale_;

    const int y = height() - 18;
    const int x = 14;
    p.setPen(QPen(QColor(220, 220, 220), 2));
    p.drawLine(QPointF(x, y), QPointF(x + px, y));
    p.drawLine(QPointF(x, y - 4), QPointF(x, y + 4));
    p.drawLine(QPointF(x + px, y - 4), QPointF(x + px, y + 4));
    QString label = meters >= 1000 ? QString("%1 km").arg(meters / 1000.0)
                                   : QString("%1 m").arg(meters);
    p.drawText(QPointF(x, y - 6), label);

    // 北向指示
    const int nx = width() - 26, ny = 26;
    p.setPen(QPen(QColor(220, 220, 220), 2));
    p.drawLine(QPointF(nx, ny + 12), QPointF(nx, ny - 12));
    p.drawLine(QPointF(nx, ny - 12), QPointF(nx - 4, ny - 6));
    p.drawLine(QPointF(nx, ny - 12), QPointF(nx + 4, ny - 6));
    p.drawText(QPointF(nx - 4, ny + 26), "N");
}

// ═══════════════════════════════════════════════════════════════════════
//  鼠标交互
// ═══════════════════════════════════════════════════════════════════════
void SceneAlignView::mousePressEvent(QMouseEvent* e) {
    lastPos_ = e->pos();

    if (e->button() == Qt::MiddleButton) {
        mode_ = Mode::PanView;
        return;
    }

    if (e->button() == Qt::RightButton) {
        if (selected_ >= 0 && items_[selected_].valid) {
            mode_ = Mode::Rotate;
            double oe, on; itemOriginEnu(items_[selected_], oe, on);
            double ce, cn; screenToEnu(e->pos(), ce, cn);
            dragPrevAngle_ = std::atan2(cn - on, ce - oe);
        }
        return;
    }

    if (e->button() == Qt::LeftButton) {
        // 「点选新增点位」模式：上报经纬度后退出该模式
        if (addPointMode_) {
            double ce, cn; screenToEnu(e->pos(), ce, cn);
            double lon, lat; conv_.toLonLat(ce, cn, lon, lat);
            setAddPointMode(false);
            emit scenePointPlaced(lon, lat);
            return;
        }
        // 优先命中场景语义点位（可选中并拖拽移动）
        int pi = hitTestPoint(e->pos());
        if (pi >= 0) {
            selectedPoint_ = scenePoints_[pi].name;
            movePointIdx_  = pi;
            mode_ = Mode::MovePoint;
            emit scenePointSelected(selectedPoint_);
            update();
            return;
        }
        int hit = hitTest(e->pos());
        if (hit >= 0) {
            selectedPoint_.clear();
            if (hit != selected_) { selected_ = hit; emit selectionChanged(selected_); }
            double oe, on; itemOriginEnu(items_[selected_], oe, on);
            double ce, cn; screenToEnu(e->pos(), ce, cn);
            if (e->modifiers() & Qt::ShiftModifier) {
                mode_ = Mode::Scale;
                dragPrevDist_ = std::max(1e-3, std::hypot(ce - oe, cn - on));
                dragStartScale_ = items_[selected_].anchor.scale;
            } else if (e->modifiers() & Qt::AltModifier) {
                mode_ = Mode::Rotate;
                dragPrevAngle_ = std::atan2(cn - on, ce - oe);
            } else {
                mode_ = Mode::Translate;
            }
        } else {
            selectedPoint_.clear();
            mode_ = Mode::PanView;
        }
        update();
    }
}

void SceneAlignView::mouseMoveEvent(QMouseEvent* e) {
    const QPointF cur = e->pos();
    const QPointF d = cur - lastPos_;

    // 实时上报光标处经纬度（用于右侧属性栏显示）
    {
        double ce, cn; screenToEnu(cur, ce, cn);
        double lon, lat; conv_.toLonLat(ce, cn, lon, lat);
        emit mouseGeoMoved(lon, lat, conv_.hasOrigin() && osm_ != nullptr);
    }

    switch (mode_) {
        case Mode::PanView: {
            viewCenterE_ -= d.x() / viewScale_;
            viewCenterN_ += d.y() / viewScale_;
            update();
            break;
        }
        case Mode::Translate: {
            if (selected_ >= 0) {
                auto& a = items_[selected_].anchor;
                double e0, n0; conv_.toEnu(a.lon, a.lat, e0, n0);
                e0 += d.x() / viewScale_;
                n0 -= d.y() / viewScale_;
                conv_.toLonLat(e0, n0, a.lon, a.lat);
                a.valid = true;
                update();
                emit anchorEdited(selected_, a);
            }
            break;
        }
        case Mode::Rotate: {
            if (selected_ >= 0) {
                auto& a = items_[selected_].anchor;
                double oe, on; conv_.toEnu(a.lon, a.lat, oe, on);
                double ce, cn; screenToEnu(cur, ce, cn);
                double ang = std::atan2(cn - on, ce - oe);
                double dAng = ang - dragPrevAngle_;
                dragPrevAngle_ = ang;
                a.headingDeg += qRadiansToDegrees(dAng);
                // 归一到 (-180, 180]
                while (a.headingDeg > 180.0)  a.headingDeg -= 360.0;
                while (a.headingDeg <= -180.0) a.headingDeg += 360.0;
                a.valid = true;
                update();
                emit anchorEdited(selected_, a);
            }
            break;
        }
        case Mode::Scale: {
            if (selected_ >= 0) {
                auto& a = items_[selected_].anchor;
                double oe, on; conv_.toEnu(a.lon, a.lat, oe, on);
                double ce, cn; screenToEnu(cur, ce, cn);
                double dist = std::max(1e-3, std::hypot(ce - oe, cn - on));
                a.scale = std::max(0.05, dragStartScale_ * (dist / dragPrevDist_));
                a.valid = true;
                update();
                emit anchorEdited(selected_, a);
            }
            break;
        }
        case Mode::MovePoint: {
            if (movePointIdx_ >= 0 && movePointIdx_ < scenePoints_.size()) {
                double ce, cn; screenToEnu(cur, ce, cn);
                double lon, lat; conv_.toLonLat(ce, cn, lon, lat);
                scenePoints_[movePointIdx_].lon = lon;
                scenePoints_[movePointIdx_].lat = lat;
                update();
                emit scenePointMoved(scenePoints_[movePointIdx_].name, lon, lat);
            }
            break;
        }
        default: break;
    }
    lastPos_ = cur;
}

void SceneAlignView::mouseReleaseEvent(QMouseEvent* e) {
    Q_UNUSED(e);
    if ((mode_ == Mode::Translate || mode_ == Mode::Rotate || mode_ == Mode::Scale)
        && selected_ >= 0) {
        emit anchorEdited(selected_, items_[selected_].anchor);
    }
    if (mode_ == Mode::MovePoint && movePointIdx_ >= 0 && movePointIdx_ < scenePoints_.size()) {
        emit scenePointMoved(scenePoints_[movePointIdx_].name,
                             scenePoints_[movePointIdx_].lon,
                             scenePoints_[movePointIdx_].lat);
    }
    movePointIdx_ = -1;
    mode_ = Mode::None;
}

void SceneAlignView::leaveEvent(QEvent* e) {
    Q_UNUSED(e);
    emit mouseGeoMoved(0.0, 0.0, false);
}

void SceneAlignView::wheelEvent(QWheelEvent* e) {
    const double factor = (e->angleDelta().y() > 0) ? 1.15 : (1.0 / 1.15);
    const QPointF pos = e->position();
    double eUnder, nUnder;
    screenToEnu(pos, eUnder, nUnder);
    viewScale_ = std::max(0.01, std::min(2000.0, viewScale_ * factor));
    // 保持光标下的点不动
    viewCenterE_ = eUnder - (pos.x() - width() * 0.5) / viewScale_;
    viewCenterN_ = nUnder + (pos.y() - height() * 0.5) / viewScale_;
    update();
}

} // namespace me
