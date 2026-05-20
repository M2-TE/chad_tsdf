#pragma once
#include "chad/detail/morton_code.hpp"

namespace chad::detail::funcs {
    // sort points by their morton code
    void inline sort(std::vector<glm::aligned_vec3>& points, float sdf_res) {
        // reciprocal of voxel resolution for later
        const float sdf_res_reciprocal = static_cast<float>(1.0 / double(sdf_res));

        // create morton codes from XYZ coordinates
        std::vector<MortonCode> morton_codes;
        morton_codes.reserve(points.size());
        for (const auto& point: points) {
            morton_codes.push_back(MortonCode{ point, sdf_res_reciprocal });
        }

        // prepare a set of indices for sorting
        std::vector<std::uint32_t> indices;
        indices.resize(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        #ifdef __cpp_lib_execution // prefer to use std::execution policy if available
        std::sort(std::execution::par, indices.begin(), indices.end(), [&](std::uint32_t a, std::uint32_t b) -> bool {
            return morton_codes[a] < morton_codes[b];
        });
        #else
        std::sort(indices.begin(), indices.end(), [&](std::uint32_t a, std::uint32_t b) -> bool {
            return morton_codes[a] < morton_codes[b];
        });
        #endif

        // sort using already sorted indices
        const auto points_copy = points;
        for (std::uint32_t i = 0; i < points.size(); i++) {
            std::uint32_t sorted_index = indices[i];
            points[i] = points_copy[sorted_index];
        }
    }
}
