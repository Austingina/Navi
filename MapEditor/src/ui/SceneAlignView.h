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

    /// 按 id 设置某张 SLAM 地图是否锁定。锁定后拖拽该地图相当于平移整个视图。
    void setItemLocked(const QString& id, bool locked);
    bool isItemLocked(const QString& id) const;

    /// 撤销上一步 SLAM 地图拖拽（geo_anchor），回退到拖拽前的状态。
    /// 返回是否成功撤销（栈为空时返回 false）。
    bool undoAnchorEdit();

    // ── 场景语义点位（OSM 底图上，经纬度 + 航向角）───────────────────
    /// 设置画布上要绘制的场景语义点位集合（拷贝，用于绘制/命中测试）。
    void setScenePoints(const QVector<SceneSemanticPoint>& pts);
    /// 设置当前高亮的场景语义点位（按名称，空串表示无）。
    void setSelectedScenePoint(const QString& name);
    /// 进入/退出「点选新增点位」模式：开启后下一次左键点击将上报经纬度。
    void setAddPointMode(bool on);
    bool addPointMode() const { return addPointMode_; }

    // ── 选择 / 视图 ─────────────────────────────────────────────────
    void setSelected(int index);
    int  selected() const { return selected_; }
    void fitView();

signals:
    /// 拖拽过程中/结束时，选中地图的 anchor 被修改。
    void anchorEdited(int index, const GeoAnchor& anchor);
    /// 通过点击画布改变了选中项（-1 表示取消选中）。
    void selectionChanged(int index);
    /// 鼠标在画布上移动时的经纬度（valid=false 表示无底图/移出画布）。
    void mouseGeoMoved(double lon, double lat, bool valid);
    /// 「新增点位」模式下点选了某处（经纬度）。
    void scenePointPlaced(double lon, double lat);
    /// 点选选中了某个场景语义点位。
    void scenePointSelected(const QString& name);
    /// 拖拽移动了某个场景语义点位（经纬度）。
    void scenePointMoved(const QString& name, double lon, double lat);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void leaveEvent(QEvent*) override;

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
        bool      locked = false;
    };

    /// 拖拽前的 anchor 快照（用于 Ctrl+Z 撤销）。
    struct AnchorUndo {
        QString   id;
        GeoAnchor anchor;
    };

    static bool buildFootprint(CloudPtr cloud, const QColor& color, SlamItem& out);

    // 坐标换算
    QPointF enuToScreen(double e, double n) const;
    void    screenToEnu(const QPointF& p, double& e, double& n) const;
    QTransform itemPixelToScreen(const SlamItem& it) const;
    QPolygonF  itemScreenPolygon(const SlamItem& it) const;
    void    itemOriginEnu(const SlamItem& it, double& e, double& n) const;
    int     hitTest(const QPointF& screenPos) const;
    int     hitTestPoint(const QPointF& screenPos) const;

    void drawOsm(QPainter& p) const;
    void drawScenePoints(QPainter& p) const;
    void drawScaleBar(QPainter& p) const;

    // 数据
    std::shared_ptr<OsmData> osm_;
    Wgs84 conv_;
    QVector<SlamItem> items_;
    int selected_ = -1;

    // 拖拽撤销栈（记录每次拖拽开始前的 anchor 快照）
    QVector<AnchorUndo> anchorUndo_;
    AnchorUndo          pendingUndo_;          // 本次拖拽开始前的快照（首次移动时才压栈）
    bool                undoArmed_ = false;    // 已记录待压栈快照
    bool                undoPushed_ = false;   // 本次拖拽已压栈
    void armAnchorUndo(int index);             // 拖拽开始：记录快照
    void commitAnchorUndo();                   // 首次移动：真正压栈

    // 场景语义点位
    QVector<SceneSemanticPoint> scenePoints_;
    QString selectedPoint_;
    bool    addPointMode_ = false;
    int     movePointIdx_ = -1;

    // 视图参数
    double viewScale_ = 4.0;      // 每米像素数
    double viewCenterE_ = 0.0;    // 屏幕中心对应的 ENU 东坐标
    double viewCenterN_ = 0.0;

    // 交互状态
    enum class Mode { None, PanView, Translate, Rotate, Scale, MovePoint };
    Mode    mode_ = Mode::None;
    QPointF lastPos_;
    double  dragPrevAngle_ = 0.0;
    double  dragPrevDist_  = 1.0;
    double  dragStartScale_ = 1.0;
};

} // namespace me
