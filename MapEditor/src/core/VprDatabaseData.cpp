#include "core/VprDatabaseData.h"

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTextStream>
#include <QDebug>
#include <QSaveFile>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace me {

namespace {

// ═══════════════════════════════════════════════════════════════════════
//  极简 NumPy .npy 读/写
//   - 仅支持小端 + non-fortran-order
//   - 支持 dtype: float32 / float64 / int64 / int32（够 vpr_preprocess 用）
//   - 1D 和 2D shape
// ═══════════════════════════════════════════════════════════════════════

constexpr uint8_t kNpyMagic[6] = {0x93, 'N', 'U', 'M', 'P', 'Y'};

struct NpyHeader {
    std::string dtype;     // 如 "<f4", "<f8", "<i8"
    bool        fortran = false;
    std::vector<size_t> shape;

    size_t numel() const {
        size_t n = 1;
        for (auto s : shape) n *= s;
        return n;
    }
    size_t itemsize() const {
        if (dtype.size() < 3) return 0;
        try { return std::stoul(dtype.substr(2)); }
        catch (...) { return 0; }
    }
};

// 极简 header 文本解析（足够覆盖 numpy 默认输出格式）
static bool parseNpyHeader(const std::string& text, NpyHeader& out, std::string& err) {
    // 期望形如：{'descr': '<f8', 'fortran_order': False, 'shape': (N, 4), }
    auto find_str = [&](const std::string& key) -> std::string {
        auto pos = text.find("'" + key + "'");
        if (pos == std::string::npos) return {};
        pos = text.find(':', pos);
        if (pos == std::string::npos) return {};
        ++pos;
        while (pos < text.size() && std::isspace((unsigned char)text[pos])) ++pos;
        return text.substr(pos);
    };

    // descr
    {
        auto v = find_str("descr");
        if (v.empty() || v.front() != '\'') { err = "missing descr"; return false; }
        auto end = v.find('\'', 1);
        if (end == std::string::npos) { err = "descr unterminated"; return false; }
        out.dtype = v.substr(1, end - 1);
    }
    // fortran_order
    {
        auto v = find_str("fortran_order");
        if (v.find("True") == 0) out.fortran = true;
        else                     out.fortran = false;
    }
    // shape
    {
        auto v = find_str("shape");
        auto lp = v.find('(');
        auto rp = v.find(')');
        if (lp == std::string::npos || rp == std::string::npos || rp <= lp) {
            err = "missing shape"; return false;
        }
        std::string inside = v.substr(lp + 1, rp - lp - 1);
        out.shape.clear();
        size_t i = 0;
        while (i < inside.size()) {
            while (i < inside.size() && (std::isspace((unsigned char)inside[i]) || inside[i] == ',')) ++i;
            if (i >= inside.size()) break;
            size_t j = i;
            while (j < inside.size() && std::isdigit((unsigned char)inside[j])) ++j;
            if (j == i) break;
            out.shape.push_back(static_cast<size_t>(std::stoul(inside.substr(i, j - i))));
            i = j;
        }
    }
    return true;
}

// 读 .npy 文件到字节数组（同时返回 header 信息）
static bool readNpy(const QString& path, NpyHeader& hdr, std::vector<uint8_t>& data,
                    std::string& err) {
    std::ifstream f(path.toStdString(), std::ios::binary);
    if (!f) { err = "open failed"; return false; }

    uint8_t magic[6];
    f.read(reinterpret_cast<char*>(magic), 6);
    if (!f || std::memcmp(magic, kNpyMagic, 6) != 0) { err = "bad magic"; return false; }

    uint8_t ver[2];
    f.read(reinterpret_cast<char*>(ver), 2);
    if (!f) { err = "short header"; return false; }

    uint32_t headerLen = 0;
    if (ver[0] == 1) {
        uint16_t hl16 = 0;
        f.read(reinterpret_cast<char*>(&hl16), 2);
        headerLen = hl16;
    } else {
        f.read(reinterpret_cast<char*>(&headerLen), 4);
    }
    if (!f) { err = "header length read failed"; return false; }

    std::string headerText(headerLen, '\0');
    f.read(headerText.data(), headerLen);
    if (!f) { err = "header text read failed"; return false; }

    if (!parseNpyHeader(headerText, hdr, err)) return false;
    if (hdr.fortran) { err = "fortran order not supported"; return false; }
    if (hdr.dtype.empty() || hdr.dtype[0] != '<') { err = "non little-endian dtype: " + hdr.dtype; return false; }

    size_t bytes = hdr.numel() * hdr.itemsize();
    data.resize(bytes);
    f.read(reinterpret_cast<char*>(data.data()), bytes);
    if (f.gcount() != static_cast<std::streamsize>(bytes)) { err = "short data"; return false; }
    return true;
}

// 写 .npy 文件（version 1.0）
static bool writeNpy(const QString& path, const std::string& dtype,
                     const std::vector<size_t>& shape,
                     const uint8_t* data, size_t bytes, std::string& err) {
    // 组 header text
    std::string shapeStr = "(";
    for (size_t i = 0; i < shape.size(); ++i) {
        shapeStr += std::to_string(shape[i]);
        // numpy 对单元素元组要求 (N,)
        if (i + 1 < shape.size() || shape.size() == 1) shapeStr += ", ";
    }
    shapeStr += ")";
    std::string headerText =
        "{'descr': '" + dtype + "', 'fortran_order': False, 'shape': " + shapeStr + ", }";

    // header 总长（含 magic 6 + ver 2 + headerLen 2 + headerText + 末尾 '\n'）
    // 要求按 64 字节对齐，padding 用空格
    size_t baseLen = 10 + headerText.size() + 1;  // 1.0 版本 headerLen 用 2 字节
    size_t pad = (64 - (baseLen % 64)) % 64;
    headerText.append(pad, ' ');
    headerText.push_back('\n');
    if (headerText.size() > 0xFFFF) { err = "header too large for npy v1"; return false; }

    QString tmpPath = path + ".tmp";
    {
        std::ofstream f(tmpPath.toStdString(), std::ios::binary | std::ios::trunc);
        if (!f) { err = "open tmp failed"; return false; }
        f.write(reinterpret_cast<const char*>(kNpyMagic), 6);
        uint8_t ver[2] = {1, 0};
        f.write(reinterpret_cast<const char*>(ver), 2);
        uint16_t hl = static_cast<uint16_t>(headerText.size());
        f.write(reinterpret_cast<const char*>(&hl), 2);
        f.write(headerText.data(), headerText.size());
        if (bytes && data) f.write(reinterpret_cast<const char*>(data), bytes);
        if (!f) { err = "write failed"; return false; }
    }
    // 原子替换
    QFile::remove(path);
    if (!QFile::rename(tmpPath, path)) {
        err = "rename failed: " + tmpPath.toStdString();
        return false;
    }
    return true;
}

} // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════
//  VprDatabaseData
// ═══════════════════════════════════════════════════════════════════════

QString VprDatabaseData::resolveImagePath(int i) const {
    if (i < 0 || i >= static_cast<int>(entries_.size())) return {};
    const QString& p = entries_[i].imagePath;
    if (p.isEmpty()) return {};
    QFileInfo fi(p);
    if (fi.isAbsolute()) return fi.absoluteFilePath();
    return QDir(dbDir_).absoluteFilePath(p);
}

bool VprDatabaseData::dirHasDatabase(const QString& dbDir) {
    return QFileInfo::exists(dbDir + "/database_meta.npz")
        && QFileInfo::exists(dbDir + "/database_poses.npy")
        && QFileInfo::exists(dbDir + "/database_image_paths.txt");
}

bool VprDatabaseData::load(const QString& dbDir, QString* errorString) {
    auto setErr = [&](const QString& s) { if (errorString) *errorString = s; };
    loaded_ = false;
    entries_.clear();
    dbDir_ = QDir(dbDir).absolutePath();

    QString posesPath = dbDir_ + "/database_poses.npy";
    QString pathsPath = dbDir_ + "/database_image_paths.txt";
    if (!QFileInfo::exists(posesPath) || !QFileInfo::exists(pathsPath)) {
        setErr(QStringLiteral("缺少 database_poses.npy 或 database_image_paths.txt"));
        return false;
    }

    // ── 1) 读 poses ────────────────────────────────────────────────
    NpyHeader hdr;
    std::vector<uint8_t> raw;
    std::string err;
    if (!readNpy(posesPath, hdr, raw, err)) {
        setErr(QString("读取 database_poses.npy 失败: %1").arg(QString::fromStdString(err)));
        return false;
    }
    if (hdr.shape.size() != 2 || hdr.shape[1] < 3) {
        setErr(QStringLiteral("database_poses.npy shape 不符 (期望 Nx4)"));
        return false;
    }
    const size_t N = hdr.shape[0];
    const size_t cols = hdr.shape[1];
    const size_t isz = hdr.itemsize();

    std::vector<VprDbEntry> tmp;
    tmp.reserve(N);
    if (hdr.dtype == "<f8" && isz == 8) {
        const double* p = reinterpret_cast<const double*>(raw.data());
        for (size_t i = 0; i < N; ++i) {
            VprDbEntry e;
            e.xyz   = {p[i * cols + 0], p[i * cols + 1], p[i * cols + 2]};
            e.ts_ns = (cols >= 4) ? p[i * cols + 3] : 0.0;
            tmp.push_back(e);
        }
    } else if (hdr.dtype == "<f4" && isz == 4) {
        const float* p = reinterpret_cast<const float*>(raw.data());
        for (size_t i = 0; i < N; ++i) {
            VprDbEntry e;
            e.xyz   = {p[i * cols + 0], p[i * cols + 1], p[i * cols + 2]};
            e.ts_ns = (cols >= 4) ? static_cast<double>(p[i * cols + 3]) : 0.0;
            tmp.push_back(e);
        }
    } else {
        setErr(QString("database_poses.npy dtype 不支持: %1").arg(QString::fromStdString(hdr.dtype)));
        return false;
    }

    // ── 2) 读 image_paths.txt ──────────────────────────────────────
    QFile f(pathsPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        setErr(QStringLiteral("打开 database_image_paths.txt 失败"));
        return false;
    }
    QTextStream ts(&f);
    ts.setCodec("UTF-8");
    QStringList paths;
    while (!ts.atEnd()) {
        QString line = ts.readLine();
        // 不 trim，因为路径可能含尾部空格（少见但保险）；只剥换行
        while (line.endsWith('\r') || line.endsWith('\n')) line.chop(1);
        paths.append(line);
    }
    f.close();
    if (static_cast<size_t>(paths.size()) != N) {
        setErr(QString("poses 行数(%1) 与 image_paths.txt 行数(%2) 不一致")
                   .arg(N).arg(paths.size()));
        return false;
    }
    for (size_t i = 0; i < N; ++i) tmp[i].imagePath = paths[static_cast<int>(i)];

    entries_ = std::move(tmp);
    loaded_  = true;
    return true;
}

bool VprDatabaseData::removeIndices(const QVector<int>& indices, QString* errorString) {
    auto setErr = [&](const QString& s) { if (errorString) *errorString = s; };
    if (!loaded_) { setErr(QStringLiteral("数据库未加载")); return false; }
    if (indices.isEmpty()) return true;

    // 1) 整理待删 set（去重 + 越界过滤）
    const int N = static_cast<int>(entries_.size());
    std::set<int> drop;
    for (int i : indices) if (i >= 0 && i < N) drop.insert(i);
    if (drop.empty()) return true;
    if (static_cast<int>(drop.size()) == N) {
        setErr(QStringLiteral("不允许删除所有 VPR 条目；请直接重建数据库"));
        return false;
    }

    // 2) 计算保留索引（顺序保持）
    std::vector<int> keep;
    keep.reserve(N - drop.size());
    for (int i = 0; i < N; ++i) if (!drop.count(i)) keep.push_back(i);

    // 3) 读 descriptors.npy（必须存在，因为线上检索要用）
    const QString descPath  = dbDir_ + "/database_descriptors.npy";
    const QString posesPath = dbDir_ + "/database_poses.npy";
    const QString pathsPath = dbDir_ + "/database_image_paths.txt";

    NpyHeader descHdr;
    std::vector<uint8_t> descRaw;
    std::string err;
    if (!readNpy(descPath, descHdr, descRaw, err)) {
        setErr(QString("读取 descriptors.npy 失败: %1").arg(QString::fromStdString(err)));
        return false;
    }
    if (descHdr.shape.size() != 2 || descHdr.shape[0] != static_cast<size_t>(N)) {
        setErr(QStringLiteral("descriptors.npy 行数与 poses 不一致"));
        return false;
    }
    const size_t D   = descHdr.shape[1];
    const size_t isz = descHdr.itemsize();
    const size_t rowBytes = D * isz;
    std::vector<uint8_t> newDescRaw(keep.size() * rowBytes);
    for (size_t i = 0; i < keep.size(); ++i) {
        std::memcpy(newDescRaw.data() + i * rowBytes,
                    descRaw.data() + static_cast<size_t>(keep[i]) * rowBytes,
                    rowBytes);
    }

    // 4) 重组 poses（按内存中的 entries_）—— 注意磁盘上 poses 可能是 4 列含 ts_ns，
    //    我们读出来时已经把 ts_ns 保留在 entries_，按它原样写回 (N, 4) float64
    std::vector<double> newPoses;
    newPoses.reserve(keep.size() * 4);
    for (int idx : keep) {
        const auto& e = entries_[idx];
        newPoses.push_back(e.xyz.x());
        newPoses.push_back(e.xyz.y());
        newPoses.push_back(e.xyz.z());
        newPoses.push_back(e.ts_ns);
    }

    // 5) 重组 image paths
    QStringList newPaths;
    newPaths.reserve(static_cast<int>(keep.size()));
    for (int idx : keep) newPaths.append(entries_[idx].imagePath);

    // 6) 写回三个文件（原子替换）。任一失败立即返回，已写入的 .tmp 由 OS 维护无副作用。
    if (!writeNpy(descPath, descHdr.dtype, {keep.size(), D},
                  newDescRaw.data(), newDescRaw.size(), err)) {
        setErr(QString("写 descriptors.npy 失败: %1").arg(QString::fromStdString(err)));
        return false;
    }
    if (!writeNpy(posesPath, "<f8", {keep.size(), 4},
                  reinterpret_cast<const uint8_t*>(newPoses.data()),
                  newPoses.size() * sizeof(double), err)) {
        setErr(QString("写 poses.npy 失败: %1").arg(QString::fromStdString(err)));
        return false;
    }
    {
        QSaveFile out(pathsPath);
        if (!out.open(QIODevice::WriteOnly | QIODevice::Text)) {
            setErr(QStringLiteral("打开 image_paths.txt 失败"));
            return false;
        }
        QTextStream ts(&out);
        ts.setCodec("UTF-8");
        for (const auto& p : newPaths) ts << p << "\n";
        if (!out.commit()) {
            setErr(QStringLiteral("写 image_paths.txt commit 失败"));
            return false;
        }
    }

    // 7) 内存同步
    std::vector<VprDbEntry> next;
    next.reserve(keep.size());
    for (int idx : keep) next.push_back(entries_[idx]);
    entries_ = std::move(next);
    return true;
}

} // namespace me
