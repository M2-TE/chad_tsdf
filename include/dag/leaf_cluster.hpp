#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <algorithm>

struct LeafCluster {
#if LEAF_BITS == 8
    typedef uint64_t ClusterValue;
#elif LEAF_BITS == 4
    typedef uint32_t ClusterValue;
#endif
    LeafCluster(ClusterValue value): _value(value) {}
    // compress 8 leaves into 64/32 bits
    LeafCluster(std::array<float, 8>& leaves): _value(0) {
        for (ClusterValue i = 0; i < 8; i++) {
            // check if leaf is valid
            if (leaves[i] > LEAF_NULL_F - 1.0f) {
                _value |= LEAF_NULL << (i * 8);
                continue;
            }

            float sd = leaves[i];
            // scale signed distance to be 1.0 for each voxel unit of distance
            sd = sd * (float)(1.0 / LEAF_RESOLUTION);
            // normalize signed distance to [-1, 1]
            sd = std::clamp(sd, -1.0f, 1.0f);
            // scale up to fit into n-bit integer
            sd = sd * (float)LEAF_RANGE;
            // add offset that values range from 0 to 255
            sd = sd + (float)LEAF_RANGE;
            // use standard rounding
            auto sd_i = (ClusterValue)sd;
            // pack the bits into the leaf cluster value
            _value |= sd_i << (i * LEAF_BITS);
        }
    }
    // void merge() {
    //     // TODO
    // }
    std::optional<float> inline get_leaf(ClusterValue index) {
        ClusterValue leaf = (_value >> (index * LEAF_BITS)) & LEAF_MASK;
        // leaf is null when all bits are set
        if (leaf == LEAF_NULL) return std::nullopt;
        // offset value to represent [-128, 127]
        float sd = ((float)leaf - (float)LEAF_RANGE);
        // scale signed distance back up to [-1, 1]
        sd *= (float)(1.0 / (double)LEAF_RANGE);
        // scale back to real-coord signed distance
        sd *= (float)LEAF_RESOLUTION;
        return std::make_optional<float>(sd);
    }

    ClusterValue _value;
    static constexpr float LEAF_NULL_F = 999.0f;
    static constexpr ClusterValue LEAF_MASK = (1 << LEAF_BITS) - 1; // mask for a single leaf
    static constexpr ClusterValue LEAF_NULL = LEAF_MASK; // leaf is null when all bits are set
    static constexpr ClusterValue LEAF_RANGE = LEAF_MASK / 2; // achievable range with data bits
};