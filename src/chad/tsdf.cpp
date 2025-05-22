#include <chrono>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <Eigen/Eigen>
#include "chad/tsdf.hpp"
#include "chad/detail/morton_code.hpp"
#include "chad/detail/normal_estimation.hpp"

namespace chad {
    TSDFMap::TSDFMap(float voxel_resolution): _voxel_resolution(voxel_resolution) {
    }
    template<> void TSDFMap::insert<glm::vec3>(const std::vector<glm::vec3>& points, const glm::vec3 position) {
        auto beg = std::chrono::high_resolution_clock::now();
        fmt::println("Inserting {} points", points.size());

        // should always calculate normals for now
        if (true) {
            // sort points by their morton code, discretized to the voxel resolution
            auto points_mc = detail::calc_mc_from_points(points, _voxel_resolution);
            auto points_sorted = detail::sort_points_by_mc(points_mc);
            // estimate the normal of every point
            auto normals = detail::estimate_normals(points_mc, position);
        }

        // insert into current submap octree
        // TODO: use 32-bit indexing, full 8-child nodes, insert signed distance and weight (double prec)

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
}