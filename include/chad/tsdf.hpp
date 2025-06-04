#pragma once
#include <vector>

// forward declare implementation details
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
        // void insert(const std::vector<std::array<float, 3>> points, std::array<float, 3> position);
        // TODO
        void save();

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
