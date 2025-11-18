#pragma once
#if __has_include(<glm/vec3.hpp>)
#   include <glm/vec3.hpp>
#endif
#if __has_include(<Eigen/Eigen>)
#   include <Eigen/Eigen>
#endif
#include <array>
#include "chad/tsdf_map.hpp"

// template specializations for TSDFMap::insert<T>
namespace chad {
    // template specialization for std::array<float, 3>
    template<>
    void inline TSDFMap::insert<std::array<float, 3>>(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        insert_pointcloud(points, position);
    }

    // template specialization for glm::vec3
    #if __has_include(<glm/vec3.hpp>)
    template<>
    void inline TSDFMap::insert<glm::vec3>(const std::vector<glm::vec3>& points, const glm::vec3& position) {
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
            insert_pointcloud(points_vec, { position.x, position.y, position.z });
        }
    }
    #endif

    // template specialization for Eigen::Vector3f
    #if __has_include(<Eigen/Eigen>)
    template<>
    void inline TSDFMap::insert<Eigen::Vector3f>(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& position) {
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
            insert_pointcloud(points_vec, { position.x(), position.y(), position.z() });
        }
    }
    #endif
}
