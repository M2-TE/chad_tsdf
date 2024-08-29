#pragma once
#include <array>
#include <cstdint>
#include <optional>

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
            if (leaves[i] == LEAF_NULL_F) {
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
            auto sd_i = (ClusterValue)sd; // use standard rounding towards 0
            // add offset that values are linear from 0 to 255
            sd_i = sd_i + LEAF_RANGE;
            // pack the bits into the leaf cluster value
            _value |= sd_i << (i * 8);
        }
    }
    void merge() {

    }
    // std::optional<float> get_leaf(uint_fast8_t index) {
    //     uint8_t mask = 0; // TODO
    //     uint8_t leaf = (_value >> (index * 8)) & 0xff;

    //     if (_value == NULL_LEAF) return std::nullopt;
    // }

    ClusterValue _value;
    static constexpr float LEAF_NULL_F = std::numeric_limits<float>::max();
    static constexpr ClusterValue LEAF_NULL = 0xff;
    static constexpr ClusterValue LEAF_MASK = (1 << LEAF_BITS) - 1; // mask for a single leaf
    static constexpr ClusterValue LEAF_RANGE = LEAF_MASK / 2; // achievable range with data bits
};