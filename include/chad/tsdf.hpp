#pragma once
#include <array>
#include <vector>
#include <string>
#include "chad/submap.hpp"

#if __has_include(<glm/vec3.hpp>)
#   include <glm/vec3.hpp>
#endif

#if __has_include(<Eigen/Eigen>)
#   include <Eigen/Eigen>
#endif

namespace chad {
    namespace detail {
        class DAG;
        struct Octree;
    }

    class TSDFMap {
    public:
        TSDFMap(const TSDFMap&  other) = delete; // copy constructor
        TSDFMap(      TSDFMap&& other) = delete; // move constructor
        TSDFMap& operator=(const TSDFMap&  other) = delete; // copy assignment
        TSDFMap& operator=(      TSDFMap&& other) = delete; // move assignment

        // initialize a TSDF map with the given voxel size and truncation distance
        TSDFMap(float sdf_res = 0.05f, float sdf_trunc = 0.1f, float submap_fin_delta = 0.5f);
        // destructor to free allocations
        ~TSDFMap();

        // Insert pointcloud alongside scanner position. VEC3 can be std::array<float, 3>, glm::vec3 or Eigen::Vector3f.
        template<typename VEC3>
        void inline insert(const std::vector<VEC3>& points, const VEC3& position);
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void inline insert(const float* points_p, size_t points_count, const float* position_p) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
            // position_p should just be x y and z
            const auto position = *reinterpret_cast<const std::array<float, 3>*>(position_p);
            insert_internal(points, position);
        }
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void inline insert(const float* points_p, size_t points_count, float x, float y, float z) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
            insert_internal(points, { x, y, z });
        }

        // finalize current active submap
        auto finalize() -> Submap;
        // DEBUG ONLY: try matching two submaps to detect loop closure
        void DEBUG_merge_submaps(const Submap& submap_a, const Submap& submap_b);
        // reconstruct 3D mesh and write it to disk
        void save(const std::string& filename);

    private:
        void insert_internal(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position);

    public:
        const float _sdf_res;
        const float _sdf_trunc;
        const float _submap_fin_delta;
        
    private:
        Submap _active_submap;
        std::vector<Submap> _submaps;

        // forward declared classes as raw pointers
        detail::DAG* _dag_p;
        detail::Octree* _active_octree_p;
    };

    // template specializations for std::array<float, 3>
    template<>
    void inline TSDFMap::insert<std::array<float, 3>>(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        insert_internal(points, position);
    }

    // template specializations for glm::vec3
    #if __has_include(<glm/vec3.hpp>)
    template<>
    void inline TSDFMap::insert<glm::vec3>(const std::vector<glm::vec3>& points, const glm::vec3& position) {
        // when using unpadded vec3, we can avoid copies
        if (sizeof(glm::vec3) == 12) {
            const float* points_p = &points[0].x;
            insert(points_p, points.size(), position.x, position.y, position.z);
        }
        else {
            std::vector<std::array<float, 3>> points_vec;
            points_vec.reserve(points.size());
            for (const auto& point: points) {
                points_vec.push_back({ point.x, point.y, point.z });
            }
            insert_internal(points_vec, { position.x, position.y, position.z });
        }
    }
    #endif

    // template specializations for Eigen::Vector3f
    #if __has_include(<Eigen/Eigen>)
    template<>
    void inline TSDFMap::insert<Eigen::Vector3f>(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position) {
        // when using unpadded Vector3f, we can avoid copies
        if (sizeof(Eigen::Vector3f) == 12 && false /*disable this temporarily*/) {
            const float* points_p = points[0].data();
            insert(points_p, points.size(), position.x(), position.y(), position.z());
        }
        else {
            std::vector<std::array<float, 3>> points_vec;
            points_vec.reserve(points.size());
            for (const auto& point: points) {
                points_vec.push_back({ point.x(), point.y(), point.z() });
            }
            insert_internal(points_vec, { position.x(), position.y(), position.z() });
        }
    }
    #endif
}