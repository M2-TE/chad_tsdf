#pragma once
#include <vector>

// TODO: hide these includes
#include <glm/vec3.hpp>


namespace chad::detail {
    struct Submap;
    struct Octree;
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
        // TODO
        void save();
        
    private:
        // insert new points into current active submap
        void update_octree(const std::vector<glm::vec3>& points_mc, const std::vector<glm::vec3>& normals, glm::vec3 position);
        // store current active submap as DAG and reset it
        void finalize_submap();

    public:
        const float _voxel_resolution;
        const float _truncation_distance;
        
    private:
        detail::Octree* _active_octree_p;
        detail::Submap* _active_submap_p;
        detail::NodeLevels* _node_levels_p;
        std::vector<detail::Submap*> _submaps;
    };
}
