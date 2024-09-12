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
                _value |= LEAF_NULL << (i * LEAF_BITS);
                continue;
            }

            float sd = leaves[i];
            // scale signed distance to be 1.0 for each voxel unit of distance
            sd = sd * (float)(1.0 / LEAF_RANGE_F);
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
    auto inline get_leaf(ClusterValue index) -> std::optional<float>{
        ClusterValue leaf = (_value >> (index * LEAF_BITS)) & LEAF_MASK;
        // leaf is null when all bits are set
        if (leaf == LEAF_NULL) return std::nullopt;
        // offset value to represent [-128, 127]
        float sd = ((float)leaf - (float)LEAF_RANGE);
        // scale signed distance back up to [-1, 1]
        sd *= (float)(1.0 / (double)LEAF_RANGE);
        // scale back to real-coord signed distance
        sd *= (float)LEAF_RANGE_F;
        return std::make_optional<float>(sd);
    }
    auto static inline merge(LeafCluster a, LeafCluster b) -> LeafCluster {
        // check if a and b are equal already
        if (a._value == b._value) return a;
        // merge the two leaf clusters leaf by leaf
        ClusterValue result = 0;
        for (ClusterValue i = 0; i < 8; i++) {
            ClusterValue leaf_a = (a._value >> (i * LEAF_BITS)) & LEAF_MASK;
            ClusterValue leaf_b = (b._value >> (i * LEAF_BITS)) & LEAF_MASK;
            // check if leaves are equal or one of them is null
            if (leaf_a == leaf_b)         result |= leaf_a << (i * LEAF_BITS);
            else if (leaf_a == LEAF_NULL) result |= leaf_b << (i * LEAF_BITS);
            else if (leaf_b == LEAF_NULL) result |= leaf_a << (i * LEAF_BITS);
            else {
                float sd_a = a.get_leaf(i).value();
                float sd_b = b.get_leaf(i).value();
                // if one of these is around 1.0, it might be too far away for avg merge
                static constexpr float THRESHOLD = LEAF_RANGE_F * 0.98f;
                if (std::abs(sd_a) > THRESHOLD && std::abs(sd_b) < THRESHOLD) {
                    result |= leaf_b << (i * LEAF_BITS);
                }
                else if (std::abs(sd_b) > THRESHOLD && std::abs(sd_a) < THRESHOLD) {
                    result |= leaf_a << (i * LEAF_BITS);
                }
                else {
                    // average the two signed distances
                    float sd = (sd_a + sd_b) / 2.0f;
                    // scale signed distance to be 1.0 for each voxel unit of distance
                    sd = sd * (float)(1.0 / LEAF_RANGE_F);
                    // normalize signed distance to [-1, 1]
                    sd = std::clamp(sd, -1.0f, 1.0f);
                    // scale up to fit into n-bit integer
                    sd = sd * (float)LEAF_RANGE;
                    // add offset that values range from 0 to 255
                    sd = sd + (float)LEAF_RANGE;
                    // use standard rounding
                    auto sd_i = (ClusterValue)sd;
                    // pack the bits into the leaf cluster value
                    result |= sd_i << (i * LEAF_BITS);
                }
            }
        }
        return { result };
    }

    ClusterValue _value;
    static constexpr float LEAF_RANGE_F = LEAF_RESOLUTION * 1.0f;
    static constexpr float LEAF_NULL_F = 999.0f;
    static constexpr ClusterValue LEAF_MASK = (1 << LEAF_BITS) - 1; // mask for a single leaf
    static constexpr ClusterValue LEAF_NULL = LEAF_MASK; // leaf is null when all bits are set
    static constexpr ClusterValue LEAF_RANGE = LEAF_MASK / 2; // achievable range with data bits
    static constexpr ClusterValue CLUSTER_NULL = std::numeric_limits<ClusterValue>::max(); // cluster is null when all bits are set
    static_assert(LEAF_BITS == 8 || LEAF_BITS == 4, "LEAF_BITS must be 8 or 4");
};