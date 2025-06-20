#pragma once
#include <array>
#include <vector>
#include <string>

#if __has_include(<glm/vec3.hpp>)
#   include <glm/vec3.hpp>
#endif

#if __has_include(<Eigen/Eigen>)
#   include <Eigen/Eigen>
#endif

// forward declare implementation details
namespace chad {
    namespace detail {
        struct Submap;
        struct Octree;
        struct NodeLevels;
    }
}
namespace chad {
    class TSDFMap {
    public:
        TSDFMap(float sdf_res = 0.05f, float sdf_trunc = 0.1f);
        TSDFMap(float sdf_res, float sdf_trunc, const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position):
            _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
            insert(points, position);
        }
        TSDFMap(float sdf_res, float sdf_trunc, const float* points_p, size_t points_count, const float* position_p):
            _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
            insert(points_p, points_count, position_p);
        }
        TSDFMap(float sdf_res, float sdf_trunc, const float* points_p, size_t points_count, float position_x, float position_y, float position_z):
            _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
            insert(points_p, points_count, position_x, position_y, position_z);
        }
        ~TSDFMap();

        // insert pointcloud alongside scanner position
        void insert(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position);
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void insert(const float* points_p, size_t points_count, const float* position_p) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            size_t vec_count = points_count / 3;
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + vec_count);
            // position_p should just be x y and z
            const auto position = *reinterpret_cast<const std::array<float, 3>*>(position_p);
            insert(points, position);
        }
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void insert(const float* points_p, size_t points_count, float position_x, float position_y, float position_z) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            size_t vec_count = points_count / 3;
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + vec_count);
            insert(points, { position_x, position_y, position_z });
        }

        #if __has_include(<glm/vec3.hpp>)
        TSDFMap(float sdf_res, float sdf_trunc, const std::vector<glm::vec3>& points, const glm::vec3& position):
            _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
            insert(points, position);
        }
        // insert pointcloud alongside scanner position (TODO: prevent copies)
        void insert(const std::vector<glm::vec3>& points, const glm::vec3& position) {
            std::vector<std::array<float, 3>> points_vec;
            points_vec.reserve(points.size());
            for (const auto& point: points) {
                points_vec.push_back({ point.x, point.y, point.z });
            }
            insert(points_vec, { position.x, position.y, position.z });
        }
        #endif

        #if __has_include(<Eigen/Eigen>)
        TSDFMap(float sdf_res, float sdf_trunc, const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position):
            _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
            insert(points, position);
        }
        // insert pointcloud alongside scanner position (TODO: prevent copies)
        void insert(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position) {
            std::vector<std::array<float, 3>> points_vec;
            points_vec.reserve(points.size());
            for (const auto& point: points) {
                points_vec.push_back({ point.x(), point.y(), point.z() });
            }
            insert(points_vec, { position.x(), position.y(), position.z() });
        }
        #endif

        // reconstruct 3D mesh and write it to disk
        void save(const std::string& filename);

        // TODO:
        // leaf iterator
        // raycast to retrieve leaves along it + physics hit
        // map merging

    public:
        const float _sdf_res;
        const float _sdf_trunc;
        
    private:
        detail::Octree* _active_octree_p;
        detail::Submap* _active_submap_p;
        detail::NodeLevels* _node_levels_p;
        std::vector<detail::Submap*> _submaps;
    };
}
