#include "algorithms/BBS3DWrapper.h"
#include <gpu_bbs3d/bbs3d.cuh>
#include <iostream>

namespace me {

struct BBS3DWrapper::Impl {
    gpu::BBS3D bbs;
    std::vector<Eigen::Vector3f> tarPoints;
};

BBS3DWrapper::BBS3DWrapper() : impl_(std::make_unique<Impl>()) {}
BBS3DWrapper::~BBS3DWrapper() = default;

void BBS3DWrapper::setTargetPoints(const std::vector<Eigen::Vector3f>& points) {
    impl_->tarPoints = points;
}

void BBS3DWrapper::setSourcePoints(const std::vector<Eigen::Vector3f>& points) {
    impl_->bbs.set_src_points(points);
}

BBS3DResult BBS3DWrapper::localize(const BBS3DParams& params) {
    BBS3DResult result;

    auto& bbs = impl_->bbs;

    bbs.set_tar_points(impl_->tarPoints, params.minLevelRes, params.maxLevel);
    bbs.set_trans_search_range(impl_->tarPoints);
    bbs.set_angular_search_range(params.minRPY, params.maxRPY);
    bbs.set_score_threshold_percentage(params.scoreThresh);

    if (params.timeoutMs > 0) {
        bbs.enable_timeout();
        bbs.set_timeout_duration_in_msec(params.timeoutMs);
    } else {
        bbs.disable_timeout();
    }

    bbs.localize();

    result.localized    = bbs.has_localized();
    result.timedOut     = bbs.has_timed_out();
    result.elapsedMs    = bbs.get_elapsed_time();
    result.bestScore    = bbs.get_best_score();
    result.scorePercent = bbs.get_best_score_percentage();
    result.pose         = bbs.get_global_pose();

    return result;
}

} // namespace me
