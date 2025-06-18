#pragma once
#include <vector>
#include <cstdint>
#include <glm/vec3.hpp>

namespace chad::detail {
    struct Submap {
        uint32_t root_addr_tsdf;
        uint32_t root_addr_weight;
        std::vector<glm::vec3> positions;
    };
}