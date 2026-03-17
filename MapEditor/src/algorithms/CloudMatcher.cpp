#include "algorithms/CloudMatcher.h"
#include "algorithms/VoxelGridFilter.h"

#include <pcl/filters/crop_box.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#include <QDebug>
#include <algorithm>
#include <cmath>

namespace me {

// ═══════════════════════════════════════════════════════════════════════
//  设置点云
// ═══════════════════════════════════════════════════════════════════════

void CloudMatcher::setTarget(CloudPtr target) { target_ = target; }
void CloudMatcher::setSource(CloudPtr source) { source_ = source; }

// ═══════════════════════════════════════════════════════════════════════
//  从源点云中裁剪原点附近的区域
// ═══════════════════════════════════════════════════════════════════════

CloudPtr CloudMatcher::cropSource(CloudPtr source, double halfSize) {
    auto cropped = std::make_shared<CloudT>();
    pcl::CropBox<PointT> cropFilter;
    cropFilter.setInputCloud(source);
    cropFilter.setMin(Eigen::Vector4f(-halfSize, -halfSize, -halfSize, 1.0));
    cropFilter.setMax(Eigen::Vector4f( halfSize,  halfSize,  halfSize, 1.0));
    cropFilter.filter(*cropped);
    qDebug() << "裁剪源点云:" << source->size() << "->" << cropped->size()
             << " 范围: ±" << halfSize << "m";
    return cropped;
}

// ═══════════════════════════════════════════════════════════════════════
//  构建目标体素哈希表
// ═══════════════════════════════════════════════════════════════════════

void CloudMatcher::buildVoxelMap(float voxelSize) {
    voxelMap_.clear();
    voxelSize_ = voxelSize;

    if (!target_) return;

    for (const auto& pt : target_->points) {
        VoxelKey key{
            static_cast<int>(std::floor(pt.x / voxelSize)),
            static_cast<int>(std::floor(pt.y / voxelSize)),
            static_cast<int>(std::floor(pt.z / voxelSize))
        };
        voxelMap_[key]++;
    }
    qDebug() << "体素哈希表建立完成, 体素数:" << voxelMap_.size()
             << " 体素大小:" << voxelSize << "m";
}

// ═══════════════════════════════════════════════════════════════════════
//  计算变换后的匹配分数
// ═══════════════════════════════════════════════════════════════════════

double CloudMatcher::computeScore(CloudPtr source, const Eigen::Matrix4f& transform) {
    if (!source || source->empty()) return 0.0;

    auto transformed = std::make_shared<CloudT>();
    pcl::transformPointCloud(*source, *transformed, transform);

    int matchCount = 0;
    for (const auto& pt : transformed->points) {
        VoxelKey key{
            static_cast<int>(std::floor(pt.x / voxelSize_)),
            static_cast<int>(std::floor(pt.y / voxelSize_)),
            static_cast<int>(std::floor(pt.z / voxelSize_))
        };
        if (voxelMap_.count(key) > 0) {
            matchCount++;
        }
    }
    return static_cast<double>(matchCount) / static_cast<double>(source->size());
}

// ═══════════════════════════════════════════════════════════════════════
//  3D-BBS 分支定界粗搜索
// ═══════════════════════════════════════════════════════════════════════

QVector<CloudMatcher::BBSCandidate>
CloudMatcher::bbsSearch(CloudPtr croppedSource,
                        const Eigen::Vector3d& guessPos,
                        const MatchParams& params) {
    QVector<BBSCandidate> candidates;

    // 降采样源点云以加速评分
    auto srcDown = VoxelGridFilter::filter(croppedSource, params.bbsVoxelSize);

    // ── 第一轮: 粗搜索 ──────────────────────────────────────────────
    double bestScore = 0.0;
    BBSCandidate bestCoarse;

    double sx = params.searchRangeXY;
    double sz = params.searchRangeZ;
    double step1 = params.searchStepCoarse;

    for (double dx = -sx; dx <= sx; dx += step1) {
        for (double dy = -sx; dy <= sx; dy += step1) {
            for (double dz = -sz; dz <= sz; dz += step1) {
                for (double yaw = -params.yawRange;
                     yaw <= params.yawRange;
                     yaw += params.yawStep) {
                    // 构造变换矩阵
                    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                    Eigen::AngleAxisf rot(yaw, Eigen::Vector3f::UnitZ());
                    T.block<3,3>(0,0) = rot.toRotationMatrix();
                    T(0,3) = guessPos.x() + dx;
                    T(1,3) = guessPos.y() + dy;
                    T(2,3) = guessPos.z() + dz;

                    double score = computeScore(srcDown, T);
                    if (score > bestScore) {
                        bestScore = score;
                        bestCoarse = {
                            {guessPos.x() + dx, guessPos.y() + dy, guessPos.z() + dz},
                            yaw, score
                        };
                    }
                }
            }
        }
    }

    if (bestScore < 0.05) {
        return candidates;  // 没有合理匹配
    }

    // ── 第二轮: 在最佳粗搜索附近细搜索 ──────────────────────────────
    double step2 = params.searchStepFine;
    double fineRange = params.searchStepCoarse;
    double fineYawRange = params.yawStep;
    double fineYawStep = 2.0 * M_PI / 180.0;  // 2度

    for (double dx = -fineRange; dx <= fineRange; dx += step2) {
        for (double dy = -fineRange; dy <= fineRange; dy += step2) {
            for (double dz = -fineRange / 2; dz <= fineRange / 2; dz += step2) {
                for (double yaw = bestCoarse.yaw - fineYawRange;
                     yaw <= bestCoarse.yaw + fineYawRange;
                     yaw += fineYawStep) {
                    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                    Eigen::AngleAxisf rot(yaw, Eigen::Vector3f::UnitZ());
                    T.block<3,3>(0,0) = rot.toRotationMatrix();
                    T(0,3) = bestCoarse.translation.x() + dx;
                    T(1,3) = bestCoarse.translation.y() + dy;
                    T(2,3) = bestCoarse.translation.z() + dz;

                    double score = computeScore(srcDown, T);
                    candidates.append({
                        {bestCoarse.translation.x() + dx,
                         bestCoarse.translation.y() + dy,
                         bestCoarse.translation.z() + dz},
                        yaw, score
                    });
                }
            }
        }
    }

    // 按分数降序排列
    std::sort(candidates.begin(), candidates.end(),
              [](const BBSCandidate& a, const BBSCandidate& b) {
                  return a.score > b.score;
              });

    // 只保留前 N 个
    if (candidates.size() > 20)
        candidates.resize(20);

    return candidates;
}

// ═══════════════════════════════════════════════════════════════════════
//  GICP 精配准
// ═══════════════════════════════════════════════════════════════════════

MatchResult CloudMatcher::gicpRefine(CloudPtr source, CloudPtr target,
                                      const Eigen::Matrix4f& initialGuess,
                                      const MatchParams& params) {
    MatchResult result;

    // 对输入做降采样以加速 GICP
    auto srcDown = VoxelGridFilter::filter(source, 0.1f);
    auto tgtDown = VoxelGridFilter::filter(target, 0.1f);

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(srcDown);
    gicp.setInputTarget(tgtDown);
    gicp.setMaximumIterations(params.gicpMaxIter);
    gicp.setTransformationEpsilon(params.gicpEpsilon);
    gicp.setMaxCorrespondenceDistance(1.0);

    auto aligned = std::make_shared<CloudT>();
    gicp.align(*aligned, initialGuess);

    if (gicp.hasConverged()) {
        result.fitness = gicp.getFitnessScore();
        result.rmse    = std::sqrt(result.fitness);

        Eigen::Matrix4f finalT = gicp.getFinalTransformation();
        result.transform = RigidTransform::fromMatrix(finalT.cast<double>());

        if (result.fitness <= params.maxFitness) {
            result.success = true;
            result.message = QString("GICP 收敛, fitness=%1").arg(result.fitness, 0, 'f', 4);
        } else {
            result.message = QString("GICP fitness=%1 > 阈值%2, 匹配不可靠")
                                 .arg(result.fitness, 0, 'f', 4)
                                 .arg(params.maxFitness, 0, 'f', 2);
        }
    } else {
        result.message = "GICP 未收敛";
    }

    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  完整匹配流程: BBS → GICP
// ═══════════════════════════════════════════════════════════════════════

MatchResult CloudMatcher::match(const Eigen::Vector3d& guessPosition,
                                 const MatchParams& params,
                                 ProgressCallback callback) {
    MatchResult result;

    if (!target_ || !source_) {
        result.message = "缺少目标或源点云";
        return result;
    }

    // Step 1: 裁剪源点云
    if (callback) callback(5, "裁剪源点云...");
    auto croppedSrc = cropSource(source_, params.cropSize / 2.0);
    if (croppedSrc->empty()) {
        result.message = "裁剪后源点云为空";
        return result;
    }

    // Step 2: 构建目标体素表
    if (callback) callback(10, "构建目标体素哈希表...");
    buildVoxelMap(params.bbsVoxelSize);

    // Step 3: BBS 粗搜索
    if (callback) callback(20, "3D-BBS 粗搜索...");
    auto candidates = bbsSearch(croppedSrc, guessPosition, params);
    if (candidates.isEmpty()) {
        result.message = "BBS 未找到合理候选位置";
        return result;
    }

    qDebug() << "BBS 找到" << candidates.size() << "个候选, 最佳分数:" << candidates[0].score;

    // Step 4: 对 top 候选逐一 GICP 验证
    if (callback) callback(50, "GICP 精配准验证...");

    // 裁剪目标点云到搜索区域附近 (加速 GICP)
    auto tgtCropped = std::make_shared<CloudT>();
    {
        double margin = params.cropSize;
        pcl::CropBox<PointT> crop;
        crop.setInputCloud(target_);
        crop.setMin(Eigen::Vector4f(
            guessPosition.x() - margin, guessPosition.y() - margin,
            guessPosition.z() - margin, 1.0));
        crop.setMax(Eigen::Vector4f(
            guessPosition.x() + margin, guessPosition.y() + margin,
            guessPosition.z() + margin, 1.0));
        crop.filter(*tgtCropped);
    }

    int tried = 0;
    for (const auto& cand : candidates) {
        tried++;
        if (callback) {
            int progress = 50 + (tried * 40) / candidates.size();
            callback(progress, QString("验证候选 %1/%2 ...").arg(tried).arg(candidates.size()));
        }

        // 构造初始猜测矩阵
        Eigen::Matrix4f initGuess = Eigen::Matrix4f::Identity();
        Eigen::AngleAxisf rot(cand.yaw, Eigen::Vector3f::UnitZ());
        initGuess.block<3,3>(0,0) = rot.toRotationMatrix();
        initGuess(0,3) = cand.translation.x();
        initGuess(1,3) = cand.translation.y();
        initGuess(2,3) = cand.translation.z();

        auto gicpResult = gicpRefine(croppedSrc, tgtCropped, initGuess, params);
        if (gicpResult.success) {
            if (callback) callback(100, "匹配成功!");
            return gicpResult;
        }
    }

    // 所有候选都未通过 GICP 验证
    result.message = QString("所有 %1 个候选均未通过 GICP 验证 (阈值=%2)")
                         .arg(candidates.size())
                         .arg(params.maxFitness, 0, 'f', 2);
    if (callback) callback(100, result.message);
    return result;
}

} // namespace me
