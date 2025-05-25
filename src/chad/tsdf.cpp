#include <chrono>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_aligned.hpp>
#include <Eigen/Eigen>
#include "chad/tsdf.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/normals.hpp"

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

        for (size_t i = 0; i < points_mc.size(); i++) {
            const glm::aligned_vec3 normal = normals[i];
            const glm::aligned_vec3 point = points_mc[i].first;
            const detail::MortonCode mc = points_mc[i].second;
            // const glm::ivec3 point_voxel = mc.decode();

            // get all voxels along ray within truncation distance via 3D version of bresenham's line algorithm
            std::vector<glm::ivec3> traversed_voxels;
            {
                // calc ray starting position
                glm::aligned_vec3 direction = glm::normalize(point - glm::aligned_vec3(position));
                glm::aligned_vec3 start = point - direction * _truncation_distance;
                glm::aligned_vec3 stop  = point + direction * _truncation_distance;
                glm::aligned_ivec3 start_voxel = glm::floor(start * voxel_reciprocal);
                glm::aligned_ivec3 stop_voxel  = glm::floor(stop  * voxel_reciprocal);



                // fmt::println("{:.2f} {:.2f} {:.2f}", start.x, start.y, start.z);
                // fmt::println("{:5} {:5} {:5}", start_voxel.x, start_voxel.y, start_voxel.z);
            }


            // TODO

            // insert leaf into octree, simply retrieve if it already exists
            // detail::Octree::Leaf& leaf = _active_submap.insert(mc);
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("oct ins  {:.2f}", dur);
    }
}