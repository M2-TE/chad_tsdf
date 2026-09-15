#pragma once
#include <memory>
#include <vector>
#include <cstdint>

#if __has_include(<glm/vec3.hpp>)
#   include <glm/vec3.hpp>
#endif

#if __has_include(<Eigen/Eigen>)
#   include <Eigen/Eigen>
#endif

namespace chad {
    namespace detail {
        namespace map { struct Optimizer; }
        namespace dag { struct Storage; }
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
        TSDFMap(double sdf_res = 0.05, double sdf_trunc = 0.1, double submap_xyz_threshhold = 5.0, double submap_cor_threshhold = 0.95);
        // explicit destructor to free forward-declared allocations
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
                std::size_t data_bytes = points.size() * sizeof(glm::vec3);
                const std::uint8_t* data_p = reinterpret_cast<const std::uint8_t*>(points.data());
                std::array<double, 3> position_arr{ position.x, position.y, position.z };
                std::array<double, 3> rotation_arr{ rotation.x, rotation.y, rotation.z };
                insert_internal(data_p, data_bytes, data_flags, position_arr, rotation_arr);
            }
            void inline insert(const std::vector<glm::vec3>& points, const glm::vec3& position, const glm::vec3& rotation) {
                insert(points, static_cast<glm::dvec3>(position), static_cast<glm::dvec3>(rotation));
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
                std::size_t data_bytes = points.size() * sizeof(Eigen::Vector3f);
                const std::uint8_t* data_p = reinterpret_cast<const std::uint8_t*>(points.data());
                std::array<double, 3> position_arr{ position.x(), position.y(), position.z() };
                std::array<double, 3> rotation_arr{ rotation.x(), rotation.y(), rotation.z() };
                insert_internal(data_p, data_bytes, data_flags, position_arr, rotation_arr);
            }
            // insert pointcloud alongside estimated scanner position and (euler) rotation
            void inline insert(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position, const Eigen::Vector3f& rotation) {
                insert(points, Eigen::Vector3d{ position.x(), position.y(), position.z() }, Eigen::Vector3d{ rotation.x(), rotation.y(), rotation.z() });
            }
        #endif

        // insert pointcloud alongside estimated scanner position and (euler) rotation
        void inline insert(const std::uint8_t* data_p, std::size_t data_bytes, PointFlags data_flags, const std::array<double, 3>& position, const std::array<double, 3>& rotation) {
            insert_internal(data_p, data_bytes, data_flags, position, rotation);
        }

        // clear all data without deallocating memory
        void clear();
        // release all hashmap memory (useful when memory is tight, automatically called by reconstruct)
        void release_hashes();
        // rebuild all hashmaps based on existing trees to allow insertion of new data
        void rebuild_hashes();
        // print full memory footprint of different components
        void print_memory_usage();
        // reconstruct 3D mesh(es) as chunks of submeshes (see _submaps_per_chunk) and write it to disk
        void reconstruct(const std::string& foldername, bool clean_first = false);

    private:
        // insert pointcloud (internal function used by all insert(...) calls)
        void insert_internal(const std::uint8_t* data_p, std::size_t data_bytes, PointFlags data_flags, std::array<double, 3> position, std::array<double, 3> rotation);

    public:
        const double _sdf_res;
        const double _sdf_trunc;

    private:
        std::unique_ptr<struct detail::dag::Storage> _dag_p; // storage for persistent hashed nodes
        std::unique_ptr<struct detail::map::Optimizer> _map_optimizer_p; // active mapping, including loop closure detection and pose optimization
    };
}
