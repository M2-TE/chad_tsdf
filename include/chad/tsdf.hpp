#pragma once
#include <array>
#include <vector>

#include <glm/glm.hpp>

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

        // TODO
        void insert(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position);
        // TODO
        void insert(const std::vector<glm::vec3>& points, const glm::vec3& position) {
            std::vector<std::array<float, 3>> points_vec;
            points_vec.reserve(points.size());
            for (const auto& point: points) {
                points_vec.push_back({ point.x, point.y, point.z });
            }
            insert(points_vec, { position.x, position.y, position.z });
        }
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
