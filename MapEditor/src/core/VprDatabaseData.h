#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  VprDatabaseData — 一个 submap 的 VPR 数据库（poses + image_paths + descriptors）
//
//  目录约定（vpr_preprocess.py 输出）：
//    <dbDir>/database_descriptors.npy   (N, D)  float32
//    <dbDir>/database_poses.npy         (N, 4)  float64  [x, y, z, ts_ns]
//    <dbDir>/database_image_paths.txt   N 行
//    <dbDir>/database_meta.npz          method/dim/num/...
//
//  MapEditor 只需要：可视化（pose xyz）+ 点击预览（image path）+ 按索引删除（同步
//  改三件套）。descriptors 我们当成"黑盒"读出来再写回，不解释内容。
// ═══════════════════════════════════════════════════════════════════════

#include <QString>
#include <QStringList>
#include <QVector>
#include <Eigen/Core>
#include <vector>

namespace me {

/// 单条 VPR 数据库记录（仅前端可视化关心的部分）
struct VprDbEntry {
    Eigen::Vector3d xyz{0, 0, 0};  // submap 局部坐标系
    double          ts_ns = 0.0;
    QString         imagePath;     // 绝对路径或相对于 dbDir 的路径
};

/// 一个 submap 的 VPR 数据库内存表示
class VprDatabaseData {
public:
    VprDatabaseData() = default;

    /// 是否已加载有效数据（至少 poses + image_paths 同步存在且长度一致）
    bool isLoaded() const { return loaded_; }

    /// 数据库目录（绝对路径）
    QString dbDir() const { return dbDir_; }

    /// 记录数
    int size() const { return static_cast<int>(entries_.size()); }

    /// 取所有记录（const 引用，调用方只读）
    const std::vector<VprDbEntry>& entries() const { return entries_; }
    const VprDbEntry& entry(int i) const { return entries_[i]; }

    /// 把 ``imagePath`` 解析为绝对路径（相对路径相对于 ``dbDir_``）
    QString resolveImagePath(int i) const;

    // ── 加载 / 保存 ─────────────────────────────────────────────────

    /// 从目录加载。失败时 ``isLoaded()`` 仍为 false 且会写 ``errorString``。
    bool load(const QString& dbDir, QString* errorString = nullptr);

    /// 仅检查目录是否像一个 VPR 数据库（看 database_meta.npz 是否存在）。
    /// 给 UI 决定 checkbox 是否可启用用。
    static bool dirHasDatabase(const QString& dbDir);

    /// 删除若干索引并把变更同步写入磁盘（descriptors / poses / image_paths 三件套）。
    /// ``indices`` 不需要预先排序、允许重复；越界值会被忽略。
    /// 写文件采用 "写 .tmp -> rename" 原子替换。
    /// 成功返回 true 并就地更新 ``entries_``；失败时 ``errorString`` 给原因，
    /// 内存与磁盘均保持不变。
    bool removeIndices(const QVector<int>& indices, QString* errorString = nullptr);

private:
    QString                  dbDir_;
    std::vector<VprDbEntry>  entries_;
    bool                     loaded_ = false;
};

} // namespace me
