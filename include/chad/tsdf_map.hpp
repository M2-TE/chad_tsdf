#pragma once
#include <memory>
#include <vector>
#include <cstdint>
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

    using PointFlags = std::uint64_t;
    enum PointFlagBits: std::uint64_t {
        eNone     = 0,
        //
        eXYZ_F32  = 0b0000'0001,
        eXYZ_F64  = 0b0000'0010,
        eXYZW_F32 = 0b0000'0100,
        eXYZW_F64 = 0b0000'1000,
        //
        eRGB_U8   = 0b0001'0000,
        eRGB_F32  = 0b0010'0000,
        eRGBA_U8  = 0b0100'0000,
        eRGBA_F32 = 0b1000'0000,
    };

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
        // insert pointcloud alongside scanner position and (euler) rotation
        void inline insert(const std::vector<glm::vec3>& points, const glm::dvec3& position, const glm::dvec3& rotation) {
            // point layout may change according to alignment
            PointFlags data_flags = PointFlagBits::eNone;
            static_assert(sizeof(glm::vec3) <= sizeof(float) * 4); // just to be safe
            if      constexpr (sizeof(glm::vec3) == sizeof(float) * 3) data_flags = PointFlagBits::eXYZ_F32;
            else if constexpr (sizeof(glm::vec3) == sizeof(float) * 4) data_flags = PointFlagBits::eXYZW_F32;

            // points will be passed as a raw byte array to avoid aliasing violations
            const std::size_t data_bytes = points.size() * sizeof(glm::vec3);
            const uint8_t* data_p = reinterpret_cast<const uint8_t*>(points.data());
            const std::array<double, 3> position_arr{ position.x, position.y, position.z };
            const std::array<double, 3> rotation_arr{ rotation.x, rotation.y, rotation.z };
            insert_internal(data_p, data_bytes, data_flags, position_arr, rotation_arr);
        }
        #endif

        #if __has_include(<Eigen/Eigen>)
        // insert pointcloud alongside estimated scanner position and (euler) rotation
        void inline insert(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3d& position, const Eigen::Vector3d& rotation) {
            // point layout may change according to alignment
            PointFlags data_flags = PointFlagBits::eNone;
            static_assert(sizeof(Eigen::Vector3f) <= sizeof(float) * 4); // just to be safe
            if      constexpr (sizeof(Eigen::Vector3f) == sizeof(float) * 3) data_flags = PointFlagBits::eXYZ_F32;
            else if constexpr (sizeof(Eigen::Vector3f) == sizeof(float) * 4) data_flags = PointFlagBits::eXYZW_F32;

            // points will be passed as a raw byte array to avoid aliasing violations
            const std::size_t data_bytes = points.size() * sizeof(Eigen::Vector3f);
            const uint8_t* data_p = reinterpret_cast<const uint8_t*>(points.data());
            const std::array<double, 3> position_arr{ position.x(), position.y(), position.z() };
            const std::array<double, 3> rotation_arr{ rotation.x(), rotation.y(), rotation.z() };
            insert_internal(data_p, data_bytes, data_flags, position_arr, rotation_arr);
        }
        #endif

        // insert pointcloud alongside estimated scanner position and (euler) rotation
        void inline insert(const std::uint8_t* data_p, std::size_t data_bytes, PointFlags data_flags, const std::array<double, 3>& position, const std::array<double, 3>& rotation) {
            insert_internal(data_p, data_bytes, data_flags, position, rotation);
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
        // insert pointcloud (internal function used by all insert(...) calls)
        void insert_internal(const std::uint8_t* data_p, std::size_t data_bytes, PointFlags data_flags, const std::array<double, 3>& position, const std::array<double, 3>& rotation);

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
