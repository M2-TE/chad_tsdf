#include <chrono>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include "chad/detail/morton.hpp"

namespace chad::detail {
    auto calc_mc_from_points(const std::vector<std::array<float, 3>>& points, const float voxel_resolution) -> MortonVector {
        auto beg = std::chrono::high_resolution_clock::now();

        // calc reciprocal of voxel resolution for later
        const float voxel_reciprocal = float(1.0 / double(voxel_resolution));

        // generate morton codes from discretized points
        MortonVector points_mc;
        points_mc.reserve(points.size());
        for (const auto& point_arr: points) {
            glm::vec3 point { point_arr[0], point_arr[1], point_arr[2] };
            // convert to voxel coordinate and discretize with floor()
            glm::vec3 point_discretized = glm::floor(point * voxel_reciprocal);
            // create morton code from discretized integer position
            points_mc.emplace_back(point, glm::ivec3(point_discretized));
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("mc  calc {:.2f}", dur);
        return points_mc;
    }
    auto sort_points_by_mc(MortonVector& points_mc) -> std::vector<glm::vec3> {
        auto beg = std::chrono::high_resolution_clock::now();

        // sort points using morton codes
        auto mc_sorter = [](const auto& a, const auto& b){
            return a.second._value > b.second._value;
        };
        // std::sort(std::execution::par_unseq, points_mc.begin(), points_mc.end(), mc_sorter);
        std::sort(points_mc.begin(), points_mc.end(), mc_sorter);
        
        // store isolated sorted points
        std::vector<glm::vec3> points_sorted;
        points_sorted.reserve(points_mc.size());
        for (const auto& mc_point: points_mc) {
            points_sorted.push_back(mc_point.first);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("mc  sort {:.2f}", dur);
        return points_sorted;
    }
}