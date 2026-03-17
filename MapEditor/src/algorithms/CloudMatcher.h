#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  CloudMatcher — 3D-BBS 粗匹配 + GICP 精配准
// ═══════════════════════════════════════════════════════════════════════

#include "core/Types.h"
#include <Eigen/Core>
#include <functional>

namespace me {

/// 匹配结果
struct MatchResult {
    bool              success   = false;
    RigidTransform    transform;
    double            fitness   = 1e9;   // GICP fitness score
    double            rmse      = 1e9;   // GICP RMSE
    QString           message;
};

/// 匹配参数
struct MatchParams {
    // BBS 搜索范围 (以假想位置为中心)
    double searchRangeXY   = 5.0;    // 米
    double searchRangeZ    = 2.0;    // 米
    double searchStepCoarse= 1.0;    // 粗搜索步长 (米)
    double searchStepFine  = 0.25;   // 细搜索步长 (米)
    double yawRange        = M_PI;   // 偏航搜索范围 (弧度)
    double yawStep         = 10.0 * M_PI / 180.0;  // 偏航搜索步长

    // 源点云裁剪
    double cropSize        = 20.0;   // 从 source 原点截取的立方体半径

    // GICP 参数
    int    gicpMaxIter     = 50;
    double gicpEpsilon     = 1e-6;
    double maxFitness      = 0.2;    // 允许的最大 fitness 阈值

    // BBS 体素
    float  bbsVoxelSize    = 0.5;    // BBS 目标体素大小
};

/// 进度回调: (progress 0-100, message)
using ProgressCallback = std::function<void(int, const QString&)>;

class CloudMatcher {
public:
    CloudMatcher() = default;

    /// 设置目标点云 (default_submap 的点云)
    void setTarget(CloudPtr target);

    /// 设置源点云 (新 submap 的点云, 将被裁剪)
    void setSource(CloudPtr source);

    /// 执行匹配
    /// @param guessPosition 用户在 default_submap 上选择的假想位置
    /// @param params        匹配参数
    /// @param callback      进度回调
    MatchResult match(const Eigen::Vector3d& guessPosition,
                      const MatchParams& params = {},
                      ProgressCallback callback = nullptr);

private:
    /// 从源点云中截取原点附近的点
    CloudPtr cropSource(CloudPtr source, double halfSize);

    /// 构建目标体素哈希表
    void buildVoxelMap(float voxelSize);

    /// 计算给定变换下的匹配分数 (匹配体素数 / 源体素数)
    double computeScore(CloudPtr source, const Eigen::Matrix4f& transform);

    /// BBS 粗搜索
    struct BBSCandidate {
        Eigen::Vector3d translation;
        double          yaw;
        double          score;
    };
    QVector<BBSCandidate> bbsSearch(CloudPtr croppedSource,
                                     const Eigen::Vector3d& guessPos,
                                     const MatchParams& params);

    /// GICP 精配准
    MatchResult gicpRefine(CloudPtr source, CloudPtr target,
                           const Eigen::Matrix4f& initialGuess,
                           const MatchParams& params);

    CloudPtr target_;
    CloudPtr source_;

    // 目标点云体素哈希表: voxelKey → 点数
    struct VoxelKey {
        int x, y, z;
        bool operator==(const VoxelKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& k) const {
            size_t h = std::hash<int>()(k.x);
            h ^= std::hash<int>()(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>()(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };
    std::unordered_map<VoxelKey, int, VoxelKeyHash> voxelMap_;
    float voxelSize_ = 0.5f;
};

} // namespace me
