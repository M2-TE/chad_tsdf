#pragma once
#include <vector>
#include <string>
#include "chad/indices.hpp"

#if __has_include(<glm/vec3.hpp>) || defined(CHAD_FORCE_GLM)
#   include <glm/vec3.hpp>
#endif

#if __has_include(<Eigen/Eigen>) || defined(CHAD_FORCE_EIGEN)
#   include <Eigen/Eigen>
#endif

namespace ndd {
};

namespace chad {
    namespace detail {
        struct Octree;
        struct DAGStorage;
        struct MapOptimizer;
    }

    class TSDFMap {
    public:
        TSDFMap(const TSDFMap&  other) = delete; // copy constructor
        TSDFMap(      TSDFMap&& other) = delete; // move constructor
        TSDFMap& operator=(const TSDFMap&  other) = delete; // copy assignment
        TSDFMap& operator=(      TSDFMap&& other) = delete; // move assignment

        // initialize a TSDF map with the given voxel size and truncation distance
        TSDFMap(float sdf_res = 0.05f, float sdf_trunc = 0.1f, float submap_fin_delta = 5.0f);
        // destructor to free allocations
        ~TSDFMap();

        #if __has_include(<glm/vec3.hpp>) || defined(CHAD_FORCE_GLM)
        // insert pointcloud alongside scanner position
        void inline insert(const std::vector<glm::vec3>& points, const glm::vec3& position) {
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
                insert_pointcloud(points_vec, { position.x, position.y, position.z });
            }
        }
        #endif
        
        #if __has_include(<Eigen/Eigen>) || defined(CHAD_FORCE_EIGEN)
        // insert pointcloud alongside scanner position
        void inline insert(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position) {
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
                insert_pointcloud(points_vec, { position.x(), position.y(), position.z() });
            }
        }
        #endif

        // insert pointcloud alongside scanner position
        void inline insert(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
            insert_pointcloud(points, position);
        }
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void inline insert(const float* points_p, std::size_t points_count, const float* position_p) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
            // position_p should just be x y and z
            const auto position = *reinterpret_cast<const std::array<float, 3>*>(position_p);
            insert_pointcloud(points, position);
        }
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void inline insert(const float* points_p, std::size_t points_count, float x, float y, float z) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
            insert_pointcloud(points, { x, y, z });
        }

        // finalize current active submap and octree
        void finalize_active_submap();
        // reconstruct 3D mesh and write it to disk
        void reconstruct(const std::string& filename);
        // reconstruct 3D mesh and write it to disk
        void reconstruct(const std::string& filename, SubmapIndex submap_handle);

    private:
        // insert points into currently active octree (internal function used by all insert(...) funcs)
        void insert_pointcloud(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position);
        // insert finalized octree as read-only tree of hashed nodes
        auto insert_octree(detail::Octree* octree_p) -> RootIndices;

    public:
        const float _sdf_res;
        const float _sdf_trunc;
        const float _submap_fin_delta;

    private:
        // transient
        ScanIndex _active_scan_start = 0; // first scan index of active submap
        ScanIndex _active_scan_final = 0; // final scan index of active submap
        detail::Octree* _active_octree_p; // currently active octree storing TSDF voxels
        // persistent
        detail::DAGStorage*   _dag_storage_p; // storage for all hashed nodes
        detail::MapOptimizer* _map_optimizer_p; // loop closure detection and pose optimization
    };
}
