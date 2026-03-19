#include "algorithms/CloudMatcher.h"
#include "algorithms/BBS3DWrapper.h"
#include "algorithms/VoxelGridFilter.h"

#include <pcl/filters/crop_box.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#include <QDebug>

namespace me {

// ═══════════════════════════════════════════════════════════════════════
//  设置点云
// ═══════════════════════════════════════════════════════════════════════

void CloudMatcher::setTarget(CloudPtr target) { target_ = target; }
void CloudMatcher::setSource(CloudPtr source) { source_ = source; }

// ═══════════════════════════════════════════════════════════════════════
//  从源点云中裁剪原点附近的区域
// ═══════════════════════════════════════════════════════════════════════

CloudPtr CloudMatcher::cropSource(CloudPtr source, const Eigen::Vector3d& center, double halfSize) {
    auto cropped = std::make_shared<CloudT>();
    pcl::CropBox<PointT> cropFilter;
    cropFilter.setInputCloud(source);
    Eigen::Vector4f cMin(center.x() - halfSize, center.y() - halfSize, center.z() - halfSize, 1.0);
    Eigen::Vector4f cMax(center.x() + halfSize, center.y() + halfSize, center.z() + halfSize, 1.0);
    cropFilter.setMin(cMin);
    cropFilter.setMax(cMax);
    cropFilter.filter(*cropped);
    qDebug() << "裁剪源点云:" << source->size() << "->" << cropped->size()
             << " 中心:(" << center.x() << "," << center.y() << "," << center.z()
             << ") 范围: ±" << halfSize << "m";
    return cropped;
}

// ═══════════════════════════════════════════════════════════════════════
//  PCL → Eigen 转换 (GPU 版使用 float)
// ═══════════════════════════════════════════════════════════════════════

static std::vector<Eigen::Vector3f> pclToEigenF(CloudPtr cloud) {
    std::vector<Eigen::Vector3f> points;
    if (!cloud) return points;
    points.reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        points.emplace_back(pt.x, pt.y, pt.z);
    }
    return points;
}

// ═══════════════════════════════════════════════════════════════════════
//  GICP 精配准
// ═══════════════════════════════════════════════════════════════════════

MatchResult CloudMatcher::gicpRefine(CloudPtr source, CloudPtr target,
                                      const Eigen::Matrix4f& initialGuess,
                                      const MatchParams& params) {
    MatchResult result;

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
//  完整匹配流程: 3D-BBS → GICP
// ═══════════════════════════════════════════════════════════════════════

MatchResult CloudMatcher::match(const Eigen::Vector3d& guessPosition,
                                 const MatchParams& params,
                                 ProgressCallback callback) {
    MatchResult result;

    if (!target_ || !source_) {
        result.message = "缺少目标或源点云";
        return result;
    }

    // ── Step 1: 裁剪源点云 ─────────────────────────────────────────────
    if (callback) callback(5, "裁剪源点云...");
    auto croppedSrc = cropSource(source_, params.srcCropCenter, params.cropSize / 2.0);
    if (croppedSrc->empty()) {
        result.message = "裁剪后源点云为空";
        return result;
    }

    // ── Step 2: 裁剪目标点云到搜索区域 (使用 XY/Z 搜索范围) ─────────
    if (callback) callback(10, "裁剪目标点云...");
    auto tgtCropped = std::make_shared<CloudT>();
    {
        double marginXY = params.searchRangeXY;
        double marginZ  = params.searchRangeZ;
        pcl::CropBox<PointT> crop;
        crop.setInputCloud(target_);
        crop.setMin(Eigen::Vector4f(
            guessPosition.x() - marginXY, guessPosition.y() - marginXY,
            guessPosition.z() - marginZ, 1.0));
        crop.setMax(Eigen::Vector4f(
            guessPosition.x() + marginXY, guessPosition.y() + marginXY,
            guessPosition.z() + marginZ, 1.0));
        crop.filter(*tgtCropped);
    }
    qDebug() << "裁剪目标点云:" << target_->size() << "->" << tgtCropped->size();

    if (tgtCropped->empty()) {
        result.message = "初始位置附近没有目标点云";
        return result;
    }

    // ── Step 3: 降采样 + PCL → Eigen 转换 ──────────────────────────
    if (callback) callback(15, "降采样点云...");
    auto srcDown = VoxelGridFilter::filter(croppedSrc, params.bbsDownsample);
    auto tgtDown = VoxelGridFilter::filter(tgtCropped, params.bbsDownsample);
    qDebug() << "BBS 降采样: src" << croppedSrc->size() << "->" << srcDown->size()
             << " tgt" << tgtCropped->size() << "->" << tgtDown->size();

    if (callback) callback(18, "准备 3D-BBS (GPU)...");
    auto tarPoints = pclToEigenF(tgtDown);
    auto srcPoints = pclToEigenF(srcDown);

    // ── Step 4: 3D-BBS GPU 粗搜索 ────────────────────────────────────
    if (callback) callback(20, "3D-BBS GPU 分支定界搜索...");

    BBS3DWrapper bbs;
    bbs.setTargetPoints(tarPoints);
    bbs.setSourcePoints(srcPoints);

    BBS3DParams bbsParams;
    bbsParams.minLevelRes = static_cast<float>(params.bbsMinLevelRes);
    bbsParams.maxLevel    = params.bbsMaxLevel;
    bbsParams.scoreThresh = static_cast<float>(params.bbsScoreThresh);
    bbsParams.timeoutMs   = params.bbsTimeoutMs;

    if (callback) callback(30, "3D-BBS GPU 搜索中...");
    auto bbsResult = bbs.localize(bbsParams);

    qDebug() << "3D-BBS GPU 完成: elapsed=" << bbsResult.elapsedMs << "ms"
             << " score=" << bbsResult.bestScore
             << " scorePercent=" << bbsResult.scorePercent
             << " localized=" << bbsResult.localized
             << " timed_out=" << bbsResult.timedOut;

    if (bbsResult.timedOut) {
        result.message = QString("3D-BBS 超时 (%1 ms), 未找到匹配")
                             .arg(bbsResult.elapsedMs, 0, 'f', 0);
        if (callback) callback(100, result.message);
        return result;
    }

    if (!bbsResult.localized) {
        result.message = QString("3D-BBS 未找到匹配 (最佳得分: %1%)")
                             .arg(bbsResult.scorePercent * 100, 0, 'f', 1);
        if (callback) callback(100, result.message);
        return result;
    }

    qDebug() << "3D-BBS 位姿:\n"
             << bbsResult.pose(0,0) << bbsResult.pose(0,1) << bbsResult.pose(0,2) << bbsResult.pose(0,3) << "\n"
             << bbsResult.pose(1,0) << bbsResult.pose(1,1) << bbsResult.pose(1,2) << bbsResult.pose(1,3) << "\n"
             << bbsResult.pose(2,0) << bbsResult.pose(2,1) << bbsResult.pose(2,2) << bbsResult.pose(2,3) << "\n"
             << bbsResult.pose(3,0) << bbsResult.pose(3,1) << bbsResult.pose(3,2) << bbsResult.pose(3,3);

    // ── Step 5: GICP 精配准 ──────────────────────────────────────────
    if (callback) callback(60, "GICP 精配准...");
    Eigen::Matrix4f initGuess = bbsResult.pose;
    result = gicpRefine(croppedSrc, tgtCropped, initGuess, params);

    if (result.success) {
        if (callback) callback(100, QString("匹配成功! fitness=%1, BBS耗时=%2ms")
                                       .arg(result.fitness, 0, 'f', 4)
                                       .arg(bbsResult.elapsedMs, 0, 'f', 0));
    } else {
        if (callback) callback(100, result.message);
    }

    return result;
}

} // namespace me
