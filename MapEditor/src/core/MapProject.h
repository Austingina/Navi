#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  MapProject — 多 Submap 地图工程：加载 / 保存 / 管理
// ═══════════════════════════════════════════════════════════════════════

#include "core/SubMap.h"
#include "core/Types.h"
#include <QMap>
#include <QString>
#include <QJsonObject>
#include <memory>

namespace me {

class MapProject {
public:
    MapProject() = default;

    // ── 加载 / 保存 ─────────────────────────────────────────────────
    /// 从目录加载: dirPath/dirName.json + PCD 文件
    bool load(const QString& dirPath);
    /// 保存到目录: dirPath/dirName.json + PCD 文件。
    /// 注意：保存时会把 ``buildNo_`` 自动更新为当前 epoch 秒（因此非 const）。
    bool save(const QString& dirPath);

    // ── JSON 对象级 (供场景 single-file 内联复用) ───────────────────
    /// 从 JSON 对象加载（不读磁盘 JSON）；pcdBaseDir 解析 map_file 相对路径并加载 PCD。
    bool loadFromObject(const QJsonObject& root, const QString& pcdBaseDir);
    /// 导出为 JSON 对象（含 global/submaps/routes；不写文件、不更新 build_no）。
    QJsonObject toObject() const;
    /// 把各 submap 的 PCD 写到 pcdBaseDir + map_file（供场景保存时落盘）。
    bool savePcds(const QString& pcdBaseDir) const;

    // ── 版本号 / 构建号 ──────────────────────────────────────────────
    /// 工程语义版本号，默认 "1.0"，由用户在 MapEditor 中显式修改。
    QString version() const { return version_; }
    void setVersion(const QString& v) { version_ = v; }
    /// 构建号 = 最近一次「内容有变更」的保存时刻 (YYYYMMDDHHMMSS)；首次创建未保存时为 0。
    qint64 buildNo() const { return buildNo_; }
    /// 内容指纹（对 toObject() 剔除 build_no 后哈希），用于判断「是否有更新」。
    QByteArray contentSignature() const;
    /// 保存前调用：若内容较上次(加载/保存)发生变化则把 build_no 刷新为当前时间戳，
    /// 否则保持不变。返回最终 build_no。
    qint64 refreshBuildNoIfChanged();

    // ── Submap 管理 ──────────────────────────────────────────────────
    QString defaultSubmapId() const { return defaultSubmapId_; }
    void setDefaultSubmapId(const QString& id) { defaultSubmapId_ = id; }

    QStringList submapIds() const;
    SubMap* submap(const QString& id);
    const SubMap* submap(const QString& id) const;
    void addSubmap(std::shared_ptr<SubMap> sm);
    void removeSubmap(const QString& id);
    int  submapCount() const { return static_cast<int>(submaps_.size()); }

    /// 计算 submap 到根 (default_submap) 的级联变换矩阵
    Eigen::Matrix4d submapToRootMatrix(const QString& id) const;

    /// 检查将 childId 的 parent 设为 parentId 是否会形成环
    bool wouldFormCycle(const QString& childId, const QString& parentId) const;

    /// 获取某个 submap 的直接子节点列表
    QStringList submapChildren(const QString& parentId) const;

    /// 验证 id 是否为 TF 树的根节点（无父节点且等于 defaultSubmapId）
    bool isTreeRoot(const QString& id) const;

    // ── Route 管理 ───────────────────────────────────────────────────
    QString defaultRouteId() const { return defaultRouteId_; }
    void setDefaultRouteId(const QString& name) { defaultRouteId_ = name; }

    const QMap<QString, Route>& routes() const { return routes_; }
    Route* route(const QString& name);
    /// const 版查找：不会触发隐式共享 QMap 的 detach（避免遍历时迭代器失效）。
    const Route* route(const QString& name) const;
    void addRoute(const Route& r) { routes_[r.name] = r; }
    void removeRoute(const QString& name);
    bool renameRoute(const QString& oldName, const QString& newName);

    /// 重命名某个 submap 内的语义点位，并同步更新所有路线中引用该点的 waypoint。
    /// 失败条件：submap 不存在 / oldName 不存在 / newName 已存在 / newName 为空。
    bool renameSemanticPoint(const QString& submapId,
                             const QString& oldName,
                             const QString& newName);


    // ── 项目路径 ─────────────────────────────────────────────────────
    QString projectDir() const { return projectDir_; }
    bool isDirty() const { return dirty_; }
    void setDirty(bool d = true) { dirty_ = d; }

private:
    /// 解析 root 对象里的 global/submaps/routes 到内部结构（PCD 从 projectDir_ 加载）。
    void parseRootObject(const QJsonObject& root);

    // JSON 解析助手
    Pose           parsePose(const QJsonObject& obj) const;
    QJsonObject    poseToJson(const Pose& p) const;
    RigidTransform parseTransform(const QJsonObject& obj) const;
    QJsonObject    transformToJson(const RigidTransform& t) const;

    // 加载 PCD 文件
    CloudPtr loadPcd(const QString& path) const;
    bool     savePcd(const QString& path, CloudPtr cloud) const;

    QString  projectDir_;
    QString  defaultSubmapId_;
    QString  defaultRouteId_;
    QString  version_ = "1.0";   // 新工程默认 "1.0"
    qint64   buildNo_ = 0;       // 内容有更新时才在保存时刷新
    QByteArray contentHash_;     // 最近一次加载/保存时的内容指纹（剔除 build_no）
    bool     dirty_ = false;

    // 使用 ordered map 保持插入顺序
    QMap<QString, std::shared_ptr<SubMap>> submaps_;
    QMap<QString, Route>                   routes_;
};

} // namespace me
