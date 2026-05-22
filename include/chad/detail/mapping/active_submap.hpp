#pragma once
#include "chad/detail/octree.hpp"
#include "chad/detail/mapping/indices.hpp"

namespace chad::detail::mapping {
    struct ActiveSubmap {
        // accumulate points/normals until newly inserted NDD has sufficient variance to previous sub-submap
        std::vector<glm::aligned_vec3> _sub_points;
        std::vector<glm::aligned_vec3> _sub_normals;
        // every inserted scan will have an index (into mapping::Optimizer scan vector) added here
        std::vector<ScanIndex> _scan_indices;
        // finished sub-submap (_sub_points/_normals) will be inserted into this tsdf octree
        Octree _tsdf_octree;
    };
}
