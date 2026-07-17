#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  SceneAlignView — 2D 俯视对齐画布
//
//  在同一张局部 ENU 平面上叠加渲染：
//    · OSM 底图 (道路/人行道/建筑/水体…)         —— 经纬度经 Wgs84 投影
//    · 若干张 SLAM 地图的俯视足迹 (footprint)      —— 点云 XY 投影栅格化
//  支持用鼠标对选中的 SLAM 地图做 平移 / 旋转 / 缩放，实时换算为 geo_anchor
//  (origin lon/lat + heading_deg + scale)，实现「把 SLAM 地图对齐到 OSM 底图」。
//
//  交互：
//    · 左键拖拽 SLAM 足迹      → 平移         (改 anchor.origin)
//    · 右键拖拽 / Alt+左键     → 绕自身原点旋转 (改 anchor.heading_deg)
//    · Shift+左键拖拽          → 缩放          (改 anchor.scale)
//    · 左键空白处拖拽 / 中键   → 平移整个视图
//    · 滚轮                    → 视图缩放(以光标为中心)
// ═══════════════════════════════════════════════════════════════════════

#include "core/Types.h"
#include "core/OsmData.h"
#include "core/SceneModel.h"
#include "core/Wgs84.h"

#include <QWidget>
#include <QImage>
#include <QColor>
#include <QVector>
#include <QPointF>
#include <memory>

namespace me {

class SceneAlignView : public QWidget {
    Q_OBJECT
public:
    explicit SceneAlignView(QWidget* parent = nullptr);

    // ── OSM 底图 ─────────────────────────────────────────────────────
    /// 设置底图与场景 ENU 原点（经纬度换算基准）。传空 osm 表示清除底图。
    void setOsm(std::shared_ptr<OsmData> osm, double originLon, double originLat);
    void setOrigin(double originLon, double originLat);

    // ── SLAM 足迹 ────────────────────────────────────────────────────
    /// 用 root 系点云为一张 SLAM 地图生成俯视足迹并加入画布。返回条目索引。
    int  addSlamItem(const QString& id, const QString& name,
                     CloudPtr rootCloud, const QColor& color,
                     const GeoAnchor& anchor);
    void updateAnchor(int index, const GeoAnchor& anchor);
    GeoAnchor anchor(int index) const;
    void removeItem(int index);
    void clearItems();
    int  itemCount() const { return items_.size(); }

    /// 按 id 设置某张 SLAM 地图正交投影足迹是否显示。
    void setItemVisible(const QString& id, bool visible);
    int  indexOfItem(const QString& id) const;

    // ── 选择 / 视图 ─────────────────────────────────────────────────
    void setSelected(int index);
    int  selected() const { return selected_; }
    void fitView();

signals:
    /// 拖拽过程中/结束时，选中地图的 anchor 被修改。
    void anchorEdited(int index, const GeoAnchor& anchor);
    /// 通过点击画布改变了选中项（-1 表示取消选中）。
    void selectionChanged(int index);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    struct SlamItem {
        QString   id;
        QString   name;
        QImage    footprint;         // ARGB, 局部俯视
        double    localMinX = 0.0;   // 像素(0,0) 对应的局部 X
        double    localMaxY = 0.0;   // 像素(0,0) 对应的局部 Y
        double    pxPerM    = 1.0;
        QColor    color;
        GeoAnchor anchor;
        bool      valid = false;
        bool      visible = true;
    };

    static bool buildFootprint(CloudPtr cloud, const QColor& color, SlamItem& out);

    // 坐标换算
    QPointF enuToScreen(double e, double n) const;
    void    screenToEnu(const QPointF& p, double& e, double& n) const;
    QTransform itemPixelToScreen(const SlamItem& it) const;
    QPolygonF  itemScreenPolygon(const SlamItem& it) const;
    void    itemOriginEnu(const SlamItem& it, double& e, double& n) const;
    int     hitTest(const QPointF& screenPos) const;

    void drawOsm(QPainter& p) const;
    void drawScaleBar(QPainter& p) const;

    // 数据
    std::shared_ptr<OsmData> osm_;
    Wgs84 conv_;
    QVector<SlamItem> items_;
    int selected_ = -1;

    // 视图参数
    double viewScale_ = 4.0;      // 每米像素数
    double viewCenterE_ = 0.0;    // 屏幕中心对应的 ENU 东坐标
    double viewCenterN_ = 0.0;

    // 交互状态
    enum class Mode { None, PanView, Translate, Rotate, Scale };
    Mode    mode_ = Mode::None;
    QPointF lastPos_;
    double  dragPrevAngle_ = 0.0;
    double  dragPrevDist_  = 1.0;
    double  dragStartScale_ = 1.0;
};

} // namespace me
