#pragma once
#include <vector>

// TODO: hide these includes
#include <glm/vec3.hpp>
#include "chad/detail/octree.hpp"


namespace chad::detail {
    struct Submap;
    struct NodeLevels;
}
namespace chad {
    class TSDFMap {
    public:
        TSDFMap(float voxel_resolution = 0.05f, float _truncation_distance = 0.1f);
        ~TSDFMap();

        // insert point using std::array<float, 3>, glm::vec3 or Eigen::Vector3f
        template<typename Point>
        void insert(const std::vector<Point>& points, const Point position);
        
    private:
        // insert new points into current active submap (TODO: move outside of TSDFMap class)
        void update_submap(const std::vector<glm::vec3>& points_mc, const std::vector<glm::vec3>& normals, glm::vec3 position);
        // store current active submap as DAG and reset it
        void finalize_submap();

    public:
        const float _voxel_resolution;
        const float _truncation_distance;
        
    private:
        detail::Octree _active_submap; // TODO: make this a submap object
        std::vector<detail::Submap*> _submaps;
        detail::NodeLevels* _node_levels_p;
    };
}
