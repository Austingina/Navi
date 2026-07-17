#include "core/OsmData.h"

#include <QFile>
#include <QFileInfo>
#include <QXmlStreamReader>
#include <QByteArray>
#include <QtEndian>
#include <QPair>
#include <QDebug>
#include <cstdint>
#include <algorithm>
#include <limits>

namespace me {

void OsmData::clear() {
    nodes_.clear();
    ways_.clear();
    hasBounds_ = false;
    minLon_ = minLat_ = maxLon_ = maxLat_ = 0.0;
    sourcePath_.clear();
    nbValid_ = false;
    nbMinLon_ = nbMinLat_ = nbMaxLon_ = nbMaxLat_ = 0.0;
}

void OsmData::accumulateNodeBounds(double lon, double lat) {
    if (!nbValid_) {
        nbMinLon_ = nbMaxLon_ = lon;
        nbMinLat_ = nbMaxLat_ = lat;
        nbValid_ = true;
        return;
    }
    nbMinLon_ = std::min(nbMinLon_, lon);
    nbMinLat_ = std::min(nbMinLat_, lat);
    nbMaxLon_ = std::max(nbMaxLon_, lon);
    nbMaxLat_ = std::max(nbMaxLat_, lat);
}

OsmWayKind OsmData::classify(const QHash<QString, QString>& tags, bool& area) {
    area = tags.value("area") == "yes";

    if (tags.contains("building") || tags.contains("building:part")) {
        area = true;
        return OsmWayKind::Building;
    }
    if (tags.value("natural") == "water" || tags.contains("waterway") ||
        tags.value("landuse") == "reservoir") {
        if (tags.value("natural") == "water") area = true;
        return OsmWayKind::Water;
    }
    if (tags.contains("barrier")) {
        return OsmWayKind::Barrier;
    }
    if (tags.contains("highway")) {
        static const QVector<QString> kFoot = {
            "footway", "path", "steps", "pedestrian",
            "living_street", "corridor", "cycleway", "track"};
        const QString hw = tags.value("highway");
        return kFoot.contains(hw) ? OsmWayKind::Footway : OsmWayKind::Highway;
    }
    return OsmWayKind::Other;
}

// ═══════════════════════════════════════════════════════════════════════
//  入口：按扩展名/内容分派 XML / PBF
// ═══════════════════════════════════════════════════════════════════════
bool OsmData::load(const QString& path) {
    clear();
    sourcePath_ = path;

    const QString lower = path.toLower();
    bool isPbf = lower.endsWith(".pbf") || lower.endsWith(".osm.pbf");

    // 无明显扩展名时嗅探首字节：XML 以 '<' 开头
    if (!isPbf && !lower.endsWith(".osm")) {
        QFile f(path);
        if (f.open(QIODevice::ReadOnly)) {
            QByteArray head = f.peek(16);
            f.close();
            const QByteArray t = head.trimmed();
            if (!t.startsWith('<')) isPbf = true;   // 非 XML → 按 PBF 试
        }
    }

    bool ok = isPbf ? loadPbf(path) : loadXml(path);
    if (!ok) return false;

    if (!hasBounds_ && nbValid_) {
        minLon_ = nbMinLon_; minLat_ = nbMinLat_;
        maxLon_ = nbMaxLon_; maxLat_ = nbMaxLat_;
        hasBounds_ = true;
    }

    qDebug() << "OSM 加载完成:" << path
             << (isPbf ? "[PBF]" : "[XML]")
             << " nodes=" << nodes_.size()
             << " ways=" << ways_.size();
    return !nodes_.isEmpty();
}

// ═══════════════════════════════════════════════════════════════════════
//  OSM XML
// ═══════════════════════════════════════════════════════════════════════
bool OsmData::loadXml(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "无法打开 OSM 文件:" << path;
        return false;
    }

    QXmlStreamReader xml(&file);

    OsmWay curWay;
    QHash<QString, QString> curTags;
    bool inWay = false;

    while (!xml.atEnd()) {
        xml.readNext();

        if (xml.isStartElement()) {
            const QStringRef name = xml.name();
            const QXmlStreamAttributes attr = xml.attributes();

            if (name == QLatin1String("bounds")) {
                minLat_ = attr.value("minlat").toDouble();
                minLon_ = attr.value("minlon").toDouble();
                maxLat_ = attr.value("maxlat").toDouble();
                maxLon_ = attr.value("maxlon").toDouble();
                hasBounds_ = true;
            } else if (name == QLatin1String("node")) {
                const qint64 id = attr.value("id").toLongLong();
                OsmNode nd;
                nd.lon = attr.value("lon").toDouble();
                nd.lat = attr.value("lat").toDouble();
                nodes_.insert(id, nd);
                accumulateNodeBounds(nd.lon, nd.lat);
            } else if (name == QLatin1String("way")) {
                inWay   = true;
                curWay  = OsmWay{};
                curTags.clear();
            } else if (name == QLatin1String("nd") && inWay) {
                curWay.refs.append(attr.value("ref").toLongLong());
            } else if (name == QLatin1String("tag") && inWay) {
                curTags.insert(attr.value("k").toString(), attr.value("v").toString());
            }
        } else if (xml.isEndElement()) {
            if (xml.name() == QLatin1String("way")) {
                if (curWay.refs.size() >= 2) {
                    bool area = false;
                    curWay.kind = classify(curTags, area);
                    if (curWay.refs.first() == curWay.refs.last() &&
                        (curWay.kind == OsmWayKind::Building ||
                         curWay.kind == OsmWayKind::Water)) {
                        area = true;
                    }
                    curWay.area = area;
                    curWay.name = curTags.value("name");
                    ways_.append(curWay);
                }
                inWay = false;
            }
        }
    }

    if (xml.hasError())
        qWarning() << "OSM XML 解析错误:" << xml.errorString();

    return !nodes_.isEmpty();
}

// ═══════════════════════════════════════════════════════════════════════
//  OSM PBF（protobuf wire format + zlib）—— 自包含解码，无额外依赖
//
//  文件结构（重复）：
//    [int32 BE = BlobHeader 长度] [BlobHeader] [Blob(datasize 字节)]
//  BlobHeader: type(1,string) datasize(3,int32)
//  Blob:       raw(1,bytes) | raw_size(2,int32) | zlib_data(3,bytes)
//  解压后：OSMHeader → HeaderBlock；OSMData → PrimitiveBlock
// ═══════════════════════════════════════════════════════════════════════
namespace {

/// 极简 protobuf 读取器（仅覆盖 OSM PBF 所需的 wire type 0/2）。
class Pbf {
public:
    Pbf(const uint8_t* p, const uint8_t* end) : p_(p), end_(end) {}
    bool eof() const { return p_ >= end_; }

    uint64_t varint() {
        uint64_t r = 0; int s = 0;
        while (p_ < end_) {
            uint8_t b = *p_++;
            r |= (uint64_t)(b & 0x7f) << s;
            if (!(b & 0x80)) break;
            s += 7;
            if (s > 63) break;
        }
        return r;
    }
    int64_t svarint() {                       // zigzag
        uint64_t v = varint();
        return (int64_t)(v >> 1) ^ -(int64_t)(v & 1);
    }
    uint32_t tag() { return (uint32_t)varint(); }   // field<<3 | wiretype

    /// 读取 length-delimited，返回子区间 [s,e)，并推进游标。
    void bytes(const uint8_t*& s, const uint8_t*& e) {
        uint64_t len = varint();
        s = p_;
        e = p_ + len;
        if (e > end_) e = end_;
        p_ = e;
    }
    /// 跳过一个未知字段。
    void skip(uint32_t wt) {
        switch (wt) {
            case 0: varint(); break;
            case 1: p_ += 8; break;
            case 2: { uint64_t l = varint(); p_ += l; if (p_ > end_) p_ = end_; } break;
            case 5: p_ += 4; break;
            default: p_ = end_; break;
        }
    }

private:
    const uint8_t* p_;
    const uint8_t* end_;
};

inline uint32_t fieldNum(uint32_t tag) { return tag >> 3; }
inline uint32_t wireType(uint32_t tag) { return tag & 0x7; }

} // namespace

bool OsmData::loadPbf(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "无法打开 OSM PBF 文件:" << path;
        return false;
    }
    const QByteArray buf = file.readAll();
    file.close();

    const uint8_t* base = reinterpret_cast<const uint8_t*>(buf.constData());
    const qint64 total = buf.size();
    qint64 pos = 0;

    auto decodeCoord = [](int64_t v, int64_t off, int64_t gran) {
        return 1e-9 * (double)(off + gran * v);
    };

    while (pos + 4 <= total) {
        // 1) BlobHeader 长度 (4 字节 big-endian)
        uint32_t hdrLen = qFromBigEndian<quint32>(base + pos);
        pos += 4;
        if (hdrLen == 0 || pos + (qint64)hdrLen > total) break;

        // 2) BlobHeader
        QString blobType;
        uint32_t dataSize = 0;
        {
            Pbf r(base + pos, base + pos + hdrLen);
            while (!r.eof()) {
                uint32_t t = r.tag();
                switch (fieldNum(t)) {
                    case 1: { const uint8_t* s; const uint8_t* e;
                             r.bytes(s, e);
                             blobType = QString::fromLatin1(
                                 reinterpret_cast<const char*>(s), int(e - s)); } break;
                    case 3: dataSize = (uint32_t)r.varint(); break;
                    default: r.skip(wireType(t)); break;
                }
            }
        }
        pos += hdrLen;
        if (dataSize == 0 || pos + (qint64)dataSize > total) break;

        // 3) Blob → 解压出 block 数据
        QByteArray block;
        {
            Pbf r(base + pos, base + pos + dataSize);
            const uint8_t* rawS = nullptr; const uint8_t* rawE = nullptr;
            const uint8_t* zS = nullptr;   const uint8_t* zE = nullptr;
            uint32_t rawSize = 0;
            bool unsupported = false;
            while (!r.eof()) {
                uint32_t t = r.tag();
                switch (fieldNum(t)) {
                    case 1: r.bytes(rawS, rawE); break;          // 未压缩
                    case 2: rawSize = (uint32_t)r.varint(); break;
                    case 3: r.bytes(zS, zE); break;              // zlib
                    case 4: case 6: case 7:                      // lzma/lz4/zstd
                        unsupported = true; r.skip(wireType(t)); break;
                    default: r.skip(wireType(t)); break;
                }
            }
            if (rawS && rawE > rawS) {
                block = QByteArray(reinterpret_cast<const char*>(rawS), int(rawE - rawS));
            } else if (zS && zE > zS) {
                // qUncompress 需要 4 字节大端原始长度前缀 + 标准 zlib 流
                QByteArray z;
                z.resize(4);
                qToBigEndian<quint32>(rawSize, reinterpret_cast<uchar*>(z.data()));
                z.append(reinterpret_cast<const char*>(zS), int(zE - zS));
                block = qUncompress(z);
            } else if (unsupported) {
                qWarning() << "OSM PBF: 不支持的压缩(lzma/lz4/zstd)，请转为 zlib 压缩的 pbf";
            }
        }
        pos += dataSize;
        if (block.isEmpty()) continue;

        if (blobType != QLatin1String("OSMData") &&
            blobType != QLatin1String("OSMHeader"))
            continue;

        const uint8_t* bb = reinterpret_cast<const uint8_t*>(block.constData());
        const uint8_t* be = bb + block.size();

        if (blobType == QLatin1String("OSMHeader")) {
            // HeaderBlock.bbox (field 1) : HeaderBBox{left1,right2,top3,bottom4} 纳度(sint64)
            Pbf r(bb, be);
            while (!r.eof()) {
                uint32_t t = r.tag();
                if (fieldNum(t) == 1 && wireType(t) == 2) {
                    const uint8_t* s; const uint8_t* e; r.bytes(s, e);
                    Pbf bx(s, e);
                    int64_t left = 0, right = 0, top = 0, bottom = 0;
                    while (!bx.eof()) {
                        uint32_t bt = bx.tag();
                        switch (fieldNum(bt)) {
                            case 1: left   = bx.svarint(); break;
                            case 2: right  = bx.svarint(); break;
                            case 3: top    = bx.svarint(); break;
                            case 4: bottom = bx.svarint(); break;
                            default: bx.skip(wireType(bt)); break;
                        }
                    }
                    minLon_ = 1e-9 * left;  maxLon_ = 1e-9 * right;
                    minLat_ = 1e-9 * bottom; maxLat_ = 1e-9 * top;
                    hasBounds_ = true;
                } else {
                    r.skip(wireType(t));
                }
            }
            continue;
        }

        // ── PrimitiveBlock ────────────────────────────────────────────
        // 先扫描：stringtable / granularity / offsets / group 区间
        QVector<QByteArray> strtab;
        int64_t granularity = 100, latOff = 0, lonOff = 0;
        QVector<QPair<const uint8_t*, const uint8_t*>> groups;
        {
            Pbf r(bb, be);
            while (!r.eof()) {
                uint32_t t = r.tag();
                switch (fieldNum(t)) {
                    case 1: {   // stringtable
                        const uint8_t* s; const uint8_t* e; r.bytes(s, e);
                        Pbf st(s, e);
                        while (!st.eof()) {
                            uint32_t stt = st.tag();
                            if (fieldNum(stt) == 1 && wireType(stt) == 2) {
                                const uint8_t* ss; const uint8_t* ee; st.bytes(ss, ee);
                                strtab.append(QByteArray(
                                    reinterpret_cast<const char*>(ss), int(ee - ss)));
                            } else st.skip(wireType(stt));
                        }
                    } break;
                    case 2: {   // primitivegroup
                        const uint8_t* s; const uint8_t* e; r.bytes(s, e);
                        groups.append(qMakePair(s, e));
                    } break;
                    case 17: granularity = (int64_t)r.varint(); break;
                    case 19: latOff = (int64_t)r.varint(); break;
                    case 20: lonOff = (int64_t)r.varint(); break;
                    default: r.skip(wireType(t)); break;
                }
            }
        }

        auto strAt = [&](uint32_t idx) -> QString {
            if ((int)idx < strtab.size())
                return QString::fromUtf8(strtab[idx]);
            return QString();
        };

        // ── 逐 group 解析 ─────────────────────────────────────────────
        for (const auto& g : groups) {
            Pbf gr(g.first, g.second);
            while (!gr.eof()) {
                uint32_t t = gr.tag();
                const uint32_t fn = fieldNum(t);
                if (fn == 2 && wireType(t) == 2) {
                    // DenseNodes
                    const uint8_t* s; const uint8_t* e; gr.bytes(s, e);
                    Pbf dn(s, e);
                    QVector<int64_t> ids, lats, lons;
                    while (!dn.eof()) {
                        uint32_t dt = dn.tag();
                        const uint32_t dfn = fieldNum(dt);
                        if ((dfn == 1 || dfn == 8 || dfn == 9) && wireType(dt) == 2) {
                            const uint8_t* ps; const uint8_t* pe; dn.bytes(ps, pe);
                            Pbf pk(ps, pe);
                            QVector<int64_t>& dst = (dfn == 1) ? ids : (dfn == 8) ? lats : lons;
                            while (!pk.eof()) dst.append(pk.svarint());
                        } else {
                            dn.skip(wireType(dt));   // 跳过 keys_vals / denseinfo
                        }
                    }
                    int64_t id = 0, lat = 0, lon = 0;
                    const int n = std::min({ids.size(), lats.size(), lons.size()});
                    for (int i = 0; i < n; ++i) {
                        id  += ids[i];
                        lat += lats[i];
                        lon += lons[i];
                        OsmNode nd;
                        nd.lon = decodeCoord(lon, lonOff, granularity);
                        nd.lat = decodeCoord(lat, latOff, granularity);
                        nodes_.insert(id, nd);
                        accumulateNodeBounds(nd.lon, nd.lat);
                    }
                } else if (fn == 1 && wireType(t) == 2) {
                    // Node (非 dense)
                    const uint8_t* s; const uint8_t* e; gr.bytes(s, e);
                    Pbf nd(s, e);
                    int64_t id = 0, lat = 0, lon = 0;
                    while (!nd.eof()) {
                        uint32_t nt = nd.tag();
                        switch (fieldNum(nt)) {
                            case 1: id  = nd.svarint(); break;
                            case 8: lat = nd.svarint(); break;
                            case 9: lon = nd.svarint(); break;
                            default: nd.skip(wireType(nt)); break;
                        }
                    }
                    OsmNode o;
                    o.lon = decodeCoord(lon, lonOff, granularity);
                    o.lat = decodeCoord(lat, latOff, granularity);
                    nodes_.insert(id, o);
                    accumulateNodeBounds(o.lon, o.lat);
                } else if (fn == 3 && wireType(t) == 2) {
                    // Way
                    const uint8_t* s; const uint8_t* e; gr.bytes(s, e);
                    Pbf wy(s, e);
                    QVector<uint32_t> keys, vals;
                    OsmWay w;
                    int64_t ref = 0;
                    while (!wy.eof()) {
                        uint32_t wt = wy.tag();
                        const uint32_t wfn = fieldNum(wt);
                        if (wfn == 2 && wireType(wt) == 2) {          // keys packed
                            const uint8_t* ps; const uint8_t* pe; wy.bytes(ps, pe);
                            Pbf pk(ps, pe);
                            while (!pk.eof()) keys.append((uint32_t)pk.varint());
                        } else if (wfn == 3 && wireType(wt) == 2) {   // vals packed
                            const uint8_t* ps; const uint8_t* pe; wy.bytes(ps, pe);
                            Pbf pk(ps, pe);
                            while (!pk.eof()) vals.append((uint32_t)pk.varint());
                        } else if (wfn == 8 && wireType(wt) == 2) {   // refs packed delta
                            const uint8_t* ps; const uint8_t* pe; wy.bytes(ps, pe);
                            Pbf pk(ps, pe);
                            while (!pk.eof()) { ref += pk.svarint(); w.refs.append(ref); }
                        } else {
                            wy.skip(wireType(wt));
                        }
                    }
                    if (w.refs.size() >= 2) {
                        QHash<QString, QString> tags;
                        const int m = std::min(keys.size(), vals.size());
                        for (int i = 0; i < m; ++i)
                            tags.insert(strAt(keys[i]), strAt(vals[i]));
                        bool area = false;
                        w.kind = classify(tags, area);
                        if (w.refs.first() == w.refs.last() &&
                            (w.kind == OsmWayKind::Building || w.kind == OsmWayKind::Water))
                            area = true;
                        w.area = area;
                        w.name = tags.value("name");
                        ways_.append(w);
                    }
                } else {
                    gr.skip(wireType(t));   // relations / changesets 等
                }
            }
        }
    }

    return !nodes_.isEmpty();
}

} // namespace me
