#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include "chad/detail/level.hpp"
#include "chad/detail/octree.hpp"

namespace chad::detail {
    struct Submap {
        uint32_t _root_tsdfs;
        uint32_t _root_weights;

        // glm::vec3 _bb_min;
        // glm::vec3 _bb_max;
    };
}
namespace chad {

    struct TSDFMap {
        // Default constructor for ChadTSDF
        TSDFMap(float voxel_resolution = 0.05f, float _truncation_distance = 0.1f);

        // insert point using std::array<float, 3>, glm::vec3 or Eigen::Vector3f
        template<typename Point>
        void insert(const std::vector<Point>& points, const Point position);
        
    private:
        void update_active_submap(const std::vector<glm::vec3>& points_mc, const std::vector<glm::vec3>& normals, glm::vec3 position);
        void finalize_submap() {

        }

    private:
        const float _voxel_resolution;
        const float _truncation_distance;
        std::vector<detail::Submap> _submaps;
        std::array<detail::NodeLevel, 20> _node_levels;
        detail::LeafClusterLevel _leafcluster_level;
        detail::Octree _active_submap;
    };
}
