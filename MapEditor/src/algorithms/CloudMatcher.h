#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  CloudMatcher — 3D-BBS 粗匹配 + GICP 精配准
//  使用 cpu::BBS3D (ros2_ws/src/3rdparty/3d_bbs) 实现分支定界搜索
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

    // 源点云裁剪
    double cropSize        = 20.0;   // 从 source 原点截取的立方体边长

    // 3D-BBS GPU 参数
    float  bbsDownsample   = 0.5f;   // BBS 输入降采样体素大小 (米)
    double bbsMinLevelRes  = 1.0;    // 最细层体素分辨率 (米)
    int    bbsMaxLevel     = 6;      // 体素层级数
    double bbsScoreThresh  = 0.1;    // 最低匹配得分百分比
    int    bbsTimeoutMs    = -60000;  // 超时 (毫秒), 0 = 不超时

    // GICP 参数
    int    gicpMaxIter     = 50;
    double gicpEpsilon     = 1e-6;
    double maxFitness      = 0.2;    // 允许的最大 fitness 阈值
};

/// 进度回调: (progress 0-100, message)
using ProgressCallback = std::function<void(int, const QString&)>;

class CloudMatcher {
public:
    CloudMatcher() = default;

    void setTarget(CloudPtr target);
    void setSource(CloudPtr source);

    /// 执行匹配
    /// @param guessPosition 用户在目标子地图上选择的假想位置
    /// @param params        匹配参数
    /// @param callback      进度回调
    MatchResult match(const Eigen::Vector3d& guessPosition,
                      const MatchParams& params = {},
                      ProgressCallback callback = nullptr);

private:
    CloudPtr cropSource(CloudPtr source, double halfSize);

    /// GICP 精配准
    MatchResult gicpRefine(CloudPtr source, CloudPtr target,
                           const Eigen::Matrix4f& initialGuess,
                           const MatchParams& params);

    CloudPtr target_;
    CloudPtr source_;
};

} // namespace me
