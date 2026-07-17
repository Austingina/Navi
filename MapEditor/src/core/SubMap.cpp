#include "core/SubMap.h"
#include "algorithms/VoxelGridFilter.h"

namespace me {

SubMap::SubMap(const QString& id) : id_(id) {}

void SubMap::setCloud(CloudPtr c) {
    cloud_ = c;
    updateDisplayCloud(displayOpts_.voxelSize);
}

void SubMap::updateDisplayCloud(float voxelSize) {
    if (!cloud_ || cloud_->empty()) {
        displayCloud_ = nullptr;
        return;
    }
    displayCloud_ = VoxelGridFilter::filter(cloud_, voxelSize);
}

SemanticPoint* SubMap::semanticPoint(const QString& name) {
    auto it = semanticPoints_.find(name);
    return (it != semanticPoints_.end()) ? &it.value() : nullptr;
}

const SemanticPoint* SubMap::semanticPoint(const QString& name) const {
    auto it = semanticPoints_.constFind(name);
    return (it != semanticPoints_.constEnd()) ? &it.value() : nullptr;
}

void SubMap::removePortal(int index) {
    if (index >= 0 && index < portals_.size())
        portals_.removeAt(index);
}

} // namespace me
