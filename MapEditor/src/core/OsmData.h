#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  OsmData — 解析 OpenStreetMap `.osm` (XML) 底图
//
//  仅解析导航/展示所需的最小子集：node(经纬度) 与 way(节点序列 + 关键 tag)，
//  并按 tag 归类为 道路 / 人行道 / 建筑 / 水体 / 屏障 / 其它，供 2D 底图渲染。
//  数据仍以 经纬度 保存；投影到局部平面由调用方用 Wgs84 完成。
// ═══════════════════════════════════════════════════════════════════════

#include <QString>
#include <QHash>
#include <QVector>

namespace me {

struct OsmNode {
    double lon = 0.0;
    double lat = 0.0;
};

enum class OsmWayKind {
    Highway,    ///< 机动车道路 (highway=*)
    Footway,    ///< 人行道 / 步道 (footway/path/steps/pedestrian...)
    Building,   ///< 建筑轮廓 (building=*)
    Water,      ///< 水体 (natural=water / waterway=*)
    Barrier,    ///< 围墙 / 栅栏 (barrier=*)
    Other       ///< 其它 (仍绘制为淡色参考线)
};

struct OsmWay {
    QVector<qint64> refs;          ///< 节点 id 序列
    OsmWayKind      kind = OsmWayKind::Other;
    bool            area = false;  ///< 是否闭合面 (建筑/水体/area=yes)
    QString         name;          ///< name tag (可空)
};

class OsmData {
public:
    /// 解析 OSM 底图。按扩展名自动区分：
    ///   · `.osm`            → OSM XML
    ///   · `.pbf` / `.osm.pbf` → OSM PBF（protobuf + zlib 二进制）
    /// 成功返回 true。
    bool load(const QString& path);
    void clear();

    bool isEmpty() const { return ways_.isEmpty() && nodes_.isEmpty(); }

    const QHash<qint64, OsmNode>& nodes() const { return nodes_; }
    const QVector<OsmWay>&        ways()  const { return ways_; }

    /// 是否含经纬度范围（<bounds> 标签或由节点包围盒推算）。
    bool   hasBounds() const { return hasBounds_; }
    double minLon() const { return minLon_; }
    double minLat() const { return minLat_; }
    double maxLon() const { return maxLon_; }
    double maxLat() const { return maxLat_; }

    /// 底图中心经纬度（默认场景原点）。
    double centerLon() const { return 0.5 * (minLon_ + maxLon_); }
    double centerLat() const { return 0.5 * (minLat_ + maxLat_); }

    QString sourcePath() const { return sourcePath_; }

private:
    static OsmWayKind classify(const QHash<QString, QString>& tags, bool& area);

    bool loadXml(const QString& path);   ///< 解析 OSM XML
    bool loadPbf(const QString& path);   ///< 解析 OSM PBF
    /// 更新节点包围盒（用于无 <bounds> 时回退推算范围）。
    void accumulateNodeBounds(double lon, double lat);

    QHash<qint64, OsmNode> nodes_;
    QVector<OsmWay>        ways_;

    bool   hasBounds_ = false;
    double minLon_ = 0.0, minLat_ = 0.0, maxLon_ = 0.0, maxLat_ = 0.0;
    QString sourcePath_;

    // 节点包围盒累积（无 <bounds> 时回退用）
    bool   nbValid_ = false;
    double nbMinLon_ = 0, nbMinLat_ = 0, nbMaxLon_ = 0, nbMaxLat_ = 0;
};

} // namespace me
