#pragma once
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include "chad/indices.hpp"

#if __has_include(<glm/vec3.hpp>)
#   include <glm/vec3.hpp>
#endif

#if __has_include(<Eigen/Eigen>)
#   include <Eigen/Eigen>
#endif

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
        TSDFMap(float sdf_res = 0.05f, float sdf_trunc = 0.1f, float submap_threshhold = 5.0f);
        // destructor to free allocations
        ~TSDFMap();

        #if __has_include(<glm/vec3.hpp>)
        // // insert pointcloud alongside scanner position
        // void inline insert(const std::vector<glm::vec3>& points, const glm::vec3& position) {
        //     // when using unpadded vec3, we can avoid copies
        //     if constexpr (sizeof(glm::vec3) == sizeof(float) * 3) {
        //         const float* points_p = &points[0].x;
        //         insert(points_p, points.size(), position.x, position.y, position.z);
        //     }
        //     else {
        //         std::vector<std::array<float, 3>> points_vec;
        //         points_vec.reserve(points.size());
        //         for (const auto& point: points) {
        //             points_vec.push_back({ point.x, point.y, point.z });
        //         }
        //         insert_pointcloud(points_vec, { position.x, position.y, position.z });
        //     }
        // }
        #endif

        #if __has_include(<Eigen/Eigen>)
        // // insert pointcloud alongside scanner position
        // void inline insert(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position) {
        //     // when using unpadded Vector3f, we can avoid copies
        //     if constexpr (sizeof(Eigen::Vector3f) == sizeof(float) * 3 && false /*disable this temporarily*/) {
        //         const float* points_p = points[0].data();
        //         insert(points_p, points.size(), position.x(), position.y(), position.z());
        //     }
        //     else {
        //         std::vector<std::array<float, 3>> points_vec;
        //         points_vec.reserve(points.size());
        //         for (const auto& point: points) {
        //             points_vec.push_back({ point.x(), point.y(), point.z() });
        //         }
        //         insert_pointcloud(points_vec, { position.x(), position.y(), position.z() });
        //     }
        // }
        #endif

        // // insert pointcloud alongside scanner position
        // void inline insert(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        //     insert_pointcloud(points, position);
        // }
        // // insert pointcloud as a raw array of repeating x,y,z coordinates
        // void inline insert(const float* points_p, std::size_t points_count, const float* position_p) {
        //     const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
        //     // use points_p as the buffer for new vector (as vec_p), not requiring any copies
        //     const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
        //     // position_p should just be x y and z
        //     const auto position = *reinterpret_cast<const std::array<float, 3>*>(position_p);
        //     insert_pointcloud(points, position);
        // }
        // // insert pointcloud as a raw array of repeating x,y,z coordinates
        // void inline insert(const float* points_p, std::size_t points_count, std::array<float, 3> position, std::array<float, 3> rotation) {
        //     const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
        //     // use points_p as the buffer for new vector (as vec_p), not requiring any copies
        //     const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
        //     insert_internal(points, position, rotation);
        // }
        // // insert pointcloud as a raw array of repeating x,y,z,w coordinates (w is ignored)
        // void inline insert(const float* points_p, std::size_t points_count, std::array<float, 3> position, std::array<float, 3> rotation) {
        //     const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
        //     // use points_p as the buffer for new vector (as vec_p), not requiring any copies
        //     const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
        //     insert_internal(points, position, rotation);
        // }

        // insert pointcloud as a raw array of repeating x,y,z,w coordinates (w is ignored)
        void inline insert(const float* points_p, std::size_t points_count, std::array<float, 3> position, std::array<float, 3> rotation) {
            // insert_internal(static_cast<const std::byte*>(points_p), 5, position, rotation);
            void* p = nullptr;
            float* f = std::bit_cast<float*, void*>(p);
        }

        // checks whether a submap is currently active (i.e. latest scans not having been finalized into a submap yet)
        bool inline is_submap_active() const { return _active_scan_end != _active_scan_beg; }

        // clear all data and release memory
        void clear() { throw std::logic_error("Function not yet implemented: chad::TSDFMap::clear()"); }
        // release all hash-related memory, useful when memory is tight for reconstructions (insertions will fail until rebuild_hashes() has been called)
        void release_hashes() { throw std::logic_error("Function not yet implemented: chad::TSDFMap::release_hashes()"); }
        // rebuild all hash structures to allow insertion of new data
        void rebuild_hashes() { throw std::logic_error("Function not yet implemented: chad::TSDFMap::rebuild_hashes()"); }
        // print full memory footprint of different components
        void print_memory_usage();

        // finalize current active submap
        void finalize_active_submap();
        // reconstruct 3D mesh(es) as chunks of submeshes (see _submaps_per_chunk) and write it to disk
        void reconstruct(const std::string& foldername, uint32_t submaps_per_chunk, bool clean_first = false);

        // DEBUG
        void dothingy(std::vector<glm::vec3>& points, glm::vec3& position);

    private:
        // insert points into currently active octree (internal function used by all insert(...) funcs)
        void insert_internal(const std::byte* points_p, std::size_t points_bytes, std::array<float, 3> position, std::array<float, 3> rotation);

        // insert finalized octree as read-only tree of hashed nodes
        auto insert_octree(const std::unique_ptr<detail::Octree>& octree_p) -> RootIndices;

    public:
        const float _sdf_res;
        const float _sdf_trunc;
        const float _submap_threshhold;
        bool _debug_outputs = false;

    private:
        // transient
        ScanIndex _active_scan_beg = 0; // index of first scan for active submap
        ScanIndex _active_scan_end = 0; // past-the-end index of final scan for active submap
        std::unique_ptr<detail::Octree> _active_octree_p; // currently active octree storing TSDF voxels
        // persistent
        std::unique_ptr<detail::DAGStorage>   _dag_storage_p; // storage for all hashed nodes
        std::unique_ptr<detail::MapOptimizer> _map_optimizer_p; // loop closure detection and pose optimization
    };
}
