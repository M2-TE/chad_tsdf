#pragma once
#include <glm/glm.hpp>
#include "chad/detail/morton_code.hpp"

namespace chad::detail {
    auto estimate_normals(const MortonVector& points_mc, const glm::vec3 position) -> std::vector<glm::vec3>;
}