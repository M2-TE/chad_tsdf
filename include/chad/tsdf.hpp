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
        // insert new points into current active submap
        void update_submap(const std::vector<glm::vec3>& points_mc, const std::vector<glm::vec3>& normals, glm::vec3 position);
        // store current active submap as DAG and reset it
        void finalize_submap();

    private:
        static constexpr size_t MAX_DEPTH = 21;
        const float _voxel_resolution;
        const float _truncation_distance;
        detail::Octree _active_submap;
        std::vector<detail::Submap> _submaps;
        // 20 levels of standard nodes
        std::array<detail::NodeLevel, MAX_DEPTH-1> _node_levels;
        // 1 level of leaf clusters
        detail::LeafClusterLevel _lc_level;
    };
}
