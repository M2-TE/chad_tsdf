#include <chrono>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_aligned.hpp>
#include <Eigen/Eigen>
#include "chad/tsdf.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/normals.hpp"

namespace chad::detail {
    void print_vec(glm::vec3 vec) {
        fmt::println("{:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z);
    }
    void print_vec(glm::aligned_vec3 vec) {
        fmt::println("{:.4f} {:.4f} {:.4f}", vec.x, vec.y, vec.z);
    }
    void print_vec(glm::ivec3 vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
    void print_vec(glm::aligned_ivec3 vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
}

namespace chad {
    TSDFMap::TSDFMap(float voxel_resolution, float truncation_distance): _voxel_resolution(voxel_resolution), _truncation_distance(truncation_distance) {
    }
    template<> void TSDFMap::insert<glm::vec3>(const std::vector<glm::vec3>& points, const glm::vec3 position) {
        auto beg = std::chrono::high_resolution_clock::now();
        fmt::println("Inserting {} points", points.size());

        // sort points by their morton code, discretized to the voxel resolution
        auto points_mc = detail::calc_mc_from_points(points, _voxel_resolution);
        [[maybe_unused]] auto points_sorted = detail::sort_points_by_mc(points_mc); // TODO: eval if ret is needed
        // estimate the normal of every point
        auto normals = detail::estimate_normals(points_mc, position);

        // octree shenanigans
        update_active_submap(points_mc, normals, position);

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("total    {:.2f}", dur);
    }
    template<> void TSDFMap::insert<Eigen::Vector3f>(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f position) {
        std::vector<glm::vec3> glm_points;
        glm_points.reserve(points.size());
        for (const auto& point: points) {
            glm_points.emplace_back(point.x(), point.y(), point.z());
        }
        insert(glm_points, glm::vec3{ position.x(), position.y(), position.z() });
    }
    template<> void TSDFMap::insert<std::array<float, 3>>(const std::vector<std::array<float, 3>>& points, const std::array<float, 3> position) {
        std::vector<glm::vec3> glm_points;
        glm_points.reserve(points.size());
        for (const auto& point: points) {
            glm_points.emplace_back(point[0], point[1], point[2]);
        }
        insert(glm_points, glm::vec3{ position[0], position[1], position[2] });
    }

    void TSDFMap::update_active_submap(const detail::MortonVector& points_mc, const std::vector<glm::vec3>& normals, glm::vec3 position) {
        auto beg = std::chrono::high_resolution_clock::now();
        const float voxel_reciprocal = float(1.0 / double(_voxel_resolution));

        std::vector<glm::ivec3> traversed_voxels;
        for (size_t i = 0; i < points_mc.size(); i++) {
            const glm::aligned_vec3 normal = normals[i];
            const glm::aligned_vec3 point = points_mc[i].first;

            // get all voxels along ray within truncation distance via variant of DDA line algorithm (-> "A fast voxel traversal algorithm for ray tracing")
            // as Bresehnham's line algorithm misses some voxels
            const glm::aligned_vec3 direction = glm::normalize(point - glm::aligned_vec3(position));
            const glm::aligned_vec3 direction_recip = 1.0f / direction;
            const glm::aligned_vec3 start = point - direction * _truncation_distance;
            const glm::aligned_vec3 final = point + direction * _truncation_distance;
            const glm::aligned_vec3 voxel_start = glm::floor(start * voxel_reciprocal);
            const glm::aligned_vec3 voxel_final = glm::floor(final * voxel_reciprocal); // this + final could be optimized away
            // stepN: direction of increment for each dimension
            const glm::aligned_ivec3 voxel_step_direction = glm::sign(voxel_final - voxel_start);
            // tDeltaN: distance of "direction" needed to traverse an entire voxel width
            const glm::aligned_vec3 voxel_step_delta = _voxel_resolution * direction_recip;
            // tMaxN: distance of "direction" needed for each separate dimension to cross current voxel boundary
            glm::aligned_vec3 voxel_step_max = ((voxel_start + glm::aligned_vec3(voxel_step_direction)) * _voxel_resolution - start) * direction_recip;
            
            glm::aligned_ivec3 voxel_current = glm::ivec3(voxel_start);
            glm::aligned_ivec3 voxel_goal = glm::aligned_ivec3(voxel_final) + voxel_step_direction;

            // traverse ray within truncation distance
            traversed_voxels.push_back(voxel_current);
            while (true) {
                if (voxel_step_max.x < voxel_step_max.y) {
                    if (voxel_step_max.x < voxel_step_max.z) {
                        voxel_current.x += voxel_step_direction.x; // step in x direction
                        voxel_step_max.x += voxel_step_delta.x; // update for next voxel boundary
                        if (voxel_current.x == voxel_goal.x) break;
                    }
                    else {
                        voxel_current.z += voxel_step_direction.z; // step in z direction
                        voxel_step_max.z += voxel_step_delta.z; // update for next voxel boundary
                        if (voxel_current.z == voxel_goal.z) break;
                    }
                }
                else {
                    if (voxel_step_max.y < voxel_step_max.z) {

                        voxel_current.y += voxel_step_direction.y; // step in y direction
                        voxel_step_max.y += voxel_step_delta.y; // update for next voxel boundary
                        if (voxel_current.y == voxel_goal.y) break;
                    }
                    else {
                        voxel_current.z += voxel_step_direction.z; // step in z direction
                        voxel_step_max.z += voxel_step_delta.z; // update for next voxel boundary
                        if (voxel_current.z == voxel_goal.z) break;
                    }
                }
                traversed_voxels.push_back(voxel_current);
            }

            for (const auto& voxel: traversed_voxels) {
                _active_submap.insert(detail::MortonCode(voxel));
            }
            traversed_voxels.clear();
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("oct ins  {:.2f}", dur);
    }
}