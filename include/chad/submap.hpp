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
    };
}