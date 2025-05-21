#pragma once
#include <limits>
#include <cstdint>

namespace chad {
    struct LeafCluster {
        uint64_t _value;
        static constexpr float LEAF_NULL_F = 999.0f;
        static constexpr uint64_t LEAF_MASK = (1 << 8) - 1; // mask with all bits set
        static constexpr uint64_t LEAF_NULL = LEAF_MASK; // leaf is null when all bits are set
        static constexpr uint64_t LEAF_RANGE = LEAF_MASK / 2; // achievable range with data bits
        static constexpr uint64_t CLUSTER_NULL = std::numeric_limits<uint64_t>::max(); // cluster is null when all bits are set
    };
}