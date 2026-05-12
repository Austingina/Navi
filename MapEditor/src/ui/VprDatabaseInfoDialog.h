#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  VprDatabaseInfoDialog — 查看一个 submap 的 VPR 数据库信息
//   * 读 <dbDir>/database_build_info.json （由 vpr_preprocess.py 写入）
//   * 兼容旧版本数据库（缺 JSON 时仅显示能从文件系统派生的信息）
// ═══════════════════════════════════════════════════════════════════════

#include "ui/BuildVprDatabaseDialog.h"

#include <QDialog>
#include <QJsonObject>
#include <QString>

namespace me {

/// 已加载的 VPR 数据库构造信息（解析 database_build_info.json 得到）
struct VprDatabaseInfo {
    bool        valid = false;        // JSON 是否存在且解析成功
    QString     dbDir;                // 数据库目录的绝对路径
    QString     imageCacheDir;        // JSON 中记录的图片缓存目录（可能为空）
    QString     builtAt;              // ISO 8601 时间戳
    QString     method;               // megaloc / salad-indoor
    int         numDatabase   = 0;
    int         descriptorDim = 0;
    QJsonObject raw;                  // 原始 JSON，便于按字段读取
};

/// 解析指定数据库目录下的 build_info JSON
VprDatabaseInfo loadVprDatabaseInfo(const QString& dbDir);

/// 将已加载的 info 转回 BuildVprParams，用于"更新数据库"时预填表单
BuildVprParams toBuildVprParams(const VprDatabaseInfo& info);

class VprDatabaseInfoDialog : public QDialog {
    Q_OBJECT
public:
    VprDatabaseInfoDialog(const QString& submapId,
                          const QString& dbDir,
                          QWidget* parent = nullptr);
};

} // namespace me
