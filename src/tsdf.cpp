#include <chrono>
#include <execution>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <Eigen/Eigen>
#include "chad/tsdf.hpp"
#include "chad/morton_code.hpp"
#include "chad/normal_estimation.hpp"

namespace chad::detail {
    auto inline calc_mc_from_points(const std::vector<glm::vec3>& points, const float voxel_resolution) -> MortonVector {
        auto beg = std::chrono::high_resolution_clock::now();

        // calc reciprocal of voxel resolution for later
        const float voxel_reciprocal = float(1.0 / double(voxel_resolution));

        // generate morton codes from discretized points
        std::vector<std::pair<glm::vec3, chad::MortonCode>> points_mc;
        points_mc.reserve(points.size());
        for (const auto& point: points) {
            // convert to voxel coordinate and discretize with floor()
            glm::vec3 point_discretized = glm::floor(point * voxel_reciprocal);
            // create morton code from discretized integer position
            points_mc.emplace_back(point, glm::ivec3(point_discretized));
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("mc calc  {:.2f}", dur);
        return points_mc;
    }
    auto inline sort_points_by_mc(MortonVector& points_mc) -> std::vector<glm::vec3> {
        auto beg = std::chrono::high_resolution_clock::now();

        // sort points using morton codes
        auto mc_sorter = [](const auto& a, const auto& b){
            return a.second._value > b.second._value;
        };
        std::sort(std::execution::par_unseq, points_mc.begin(), points_mc.end(), mc_sorter);
        
        // store isolated sorted points
        std::vector<glm::vec3> points_sorted;
        points_sorted.reserve(points_mc.size());
        for (const auto& mc_point: points_mc) {
            points_sorted.push_back(mc_point.first);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("mc sort  {:.2f}", dur);
        return points_sorted;
    }
}

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