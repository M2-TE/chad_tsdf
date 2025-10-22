#pragma once
#include <array>
#include <vector>
#include <cstdint>

namespace chad {
    struct Submap {
        Submap(): root_addr_tsdf(0), root_addr_weight(0) {
        }
        
        void clear() {
            root_addr_tsdf = 0;
            root_addr_weight = 0;
            positions.clear();
        }

        uint32_t root_addr_tsdf;
        uint32_t root_addr_weight;
        std::vector<std::array<float, 3>> positions;
        std::array<float, 3> position; // avg of all positions and origin for error vectors
        std::array<float, 3> error_pos = { 0, 0, 0 };
        std::array<float, 3> error_rot_euler = { 0, 0, 0 };
    };
}