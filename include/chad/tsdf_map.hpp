#pragma once
#include <vector>
#include <string>
#include "chad/submap.hpp"

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
        TSDFMap(float sdf_res = 0.05f, float sdf_trunc = 0.1f, float submap_fin_delta = 5.0f);
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
            insert_pointcloud(points, position);
        }
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void inline insert(const float* points_p, size_t points_count, float x, float y, float z) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
            insert_pointcloud(points, { x, y, z });
        }

        // reconstruct 3D mesh and write it to disk
        void reconstruct(const std::string& filename);
        // reconstruct 3D mesh and write it to disk
        void reconstruct(const std::string& filename, Submap::Handle submap_handle);

    private:
        // insert points into currently active octree
        void insert_pointcloud(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position);
        // insert finalized octree as read-only tree of hashed nodes
        auto insert_octree(detail::Octree* octree_p) -> Submap::Roots;
        // finalize current active submap and octree
        void finalize_active_submap();

    public:
        const float _sdf_res;
        const float _sdf_trunc;
        const float _submap_fin_delta;
        const uint32_t _slice_count = 3; // TODO: not yet customizable

    private:
        using DAG = detail::DAG;
        using Octree = detail::Octree;
        // transient
        Submap _active_submap; // currently active submap storing metadata
        Octree* _active_octree_p; // currently active octree storing TSDF voxels
        // persistent
        std::vector<Submap> _submaps; // storage for all finalized submaps
        DAG* _dag_p; // storage for all hashed nodes
    };
}
