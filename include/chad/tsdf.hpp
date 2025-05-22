#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include "chad/level.hpp"

namespace chad {
    struct Submap {
        uint32_t _root_tsdfs;
        uint32_t _root_weights;

        // glm::vec3 _bb_min;
        // glm::vec3 _bb_max;
    };

    struct TSDFMap {
        // Default constructor for ChadTSDF
        TSDFMap(float voxel_resolution = 0.05f);

        // insert point using std::array<float, 3>, glm::vec3 or Eigen::Vector3f
        template<typename Point>
        void insert(const std::vector<Point>& points, const Point position);

    private:
        const float _voxel_resolution;
        std::vector<Submap> _submaps;
        std::array<NodeLevel, 20> _node_levels;
        LeafClusterLevel _leafcluster_level;
    };
}
