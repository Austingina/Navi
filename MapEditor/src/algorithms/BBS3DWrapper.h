#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  BBS3DWrapper — gpu::BBS3D 的纯 C++ 接口封装
//  隔离 CUDA 依赖, 使 CloudMatcher.cpp 不需要 nvcc 编译
// ═══════════════════════════════════════════════════════════════════════

#include <Eigen/Core>
#include <vector>
#include <memory>

namespace me {

struct BBS3DResult {
    bool   localized  = false;
    bool   timedOut   = false;
    double elapsedMs  = 0.0;
    int    bestScore  = 0;
    float  scorePercent = 0.0f;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
};

struct BBS3DParams {
    float  minLevelRes   = 1.0f;
    int    maxLevel      = 6;
    float  scoreThresh   = 0.3f;
    int    timeoutMs     = 30000;
    Eigen::Vector3f minRPY{-0.1f, -0.1f, -static_cast<float>(M_PI)};
    Eigen::Vector3f maxRPY{ 0.1f,  0.1f,  static_cast<float>(M_PI)};
};

class BBS3DWrapper {
public:
    BBS3DWrapper();
    ~BBS3DWrapper();

    void setTargetPoints(const std::vector<Eigen::Vector3f>& points);
    void setSourcePoints(const std::vector<Eigen::Vector3f>& points);

    BBS3DResult localize(const BBS3DParams& params);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace me
