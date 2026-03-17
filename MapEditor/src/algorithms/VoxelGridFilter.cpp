#include "algorithms/VoxelGridFilter.h"
#include <pcl/filters/voxel_grid.h>

namespace me {

CloudPtr VoxelGridFilter::filter(CloudPtr input, float leafSize) {
    if (!input || input->empty()) return input;
    if (leafSize <= 0.0f) return input;

    auto output = std::make_shared<CloudT>();
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*output);
    return output;
}

} // namespace me
