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
        TSDFMap(float sdf_res = 0.05f, float sdf_trunc = 0.1f);
        // initialize a TSDF map with the given voxel size and truncation distance, then insert points into it
        TSDFMap(float sdf_res, float sdf_trunc, const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position):
            _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
            insert(points, position);
        }
        // initialize a TSDF map with the given voxel size and truncation distance, then insert points into it
        TSDFMap(float sdf_res, float sdf_trunc, const float* points_p, size_t points_count, const float* position_p):
            _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
            insert(points_p, points_count, position_p);
        }
        // initialize a TSDF map with the given voxel size and truncation distance, then insert points into it
        TSDFMap(float sdf_res, float sdf_trunc, const float* points_p, size_t points_count, float position_x, float position_y, float position_z):
            _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
            insert(points_p, points_count, position_x, position_y, position_z);
        }
        ~TSDFMap();

        // insert pointcloud alongside scanner position
        void insert(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position);
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void inline insert(const float* points_p, size_t points_count, const float* position_p) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
            // position_p should just be x y and z
            const auto position = *reinterpret_cast<const std::array<float, 3>*>(position_p);
            insert(points, position);
        }
        // insert pointcloud as a raw array of repeating x,y,z coordinates
        void inline insert(const float* points_p, size_t points_count, float position_x, float position_y, float position_z) {
            const auto* vec_p = reinterpret_cast<const std::array<float, 3>*>(points_p);
            // use points_p as the buffer for new vector (as vec_p), not requiring any copies
            const auto points = std::vector<std::array<float, 3>>(vec_p, vec_p + points_count);
            insert(points, { position_x, position_y, position_z });
        }

        // create interface for GLM vectors, if the header is present
        #if __has_include(<glm/vec3.hpp>)
            // initialize a TSDF map with the given voxel size and truncation distance, then insert points into it
            TSDFMap(float sdf_res, float sdf_trunc, const std::vector<glm::vec3>& points, const glm::vec3& position):
                _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
                insert(points, position);
            }
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
                    insert(points_vec, { position.x, position.y, position.z });
                }
            }
        #endif
        
        // create interface for Eigen vectors, if the header is present
        #if __has_include(<Eigen/Eigen>)
            // initialize a TSDF map with the given voxel size and truncation distance, then insert points into it
            TSDFMap(float sdf_res, float sdf_trunc, const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position):
                _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
                insert(points, position);
            }
            // insert pointcloud alongside scanner position
            void inline insert(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position){
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
                    insert(points_vec, { position.x(), position.y(), position.z() });
                }
            }
        #endif

        // finalize current active submap
        void finalize();

        // reconstruct 3D mesh and write it to disk
        void save(const std::string& filename);

        // TODO:
        // get submaps by bounding box
        // leaf iterator
        // raycast to retrieve leaves along it + physics hit
        // map merging
    public:
        const float _sdf_res;
        const float _sdf_trunc;
        
    private:
        Submap _active_submap;
        std::vector<Submap> _submaps;

        // forward declared classes as raw pointers
        detail::DAG* _dag_p;
        detail::Octree* _active_octree_p;
    };
}