#pragma once
// ═══════════════════════════════════════════════════════════════════════
//  VoxelGridFilter — 体素降采样
// ═══════════════════════════════════════════════════════════════════════

#include "core/Types.h"

namespace me {

class VoxelGridFilter {
public:
    /// 对点云进行体素降采样, leafSize 单位为米
    static CloudPtr filter(CloudPtr input, float leafSize);
};

} // namespace me
