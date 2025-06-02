#pragma once
#include <limits>
#include <cstdint>
#include <algorithm>

namespace chad {
    struct LeafCluster {
        void set_leaf_sd(uint8_t child_i, float signed_distance, float truncation_distance_recip) {
            static constexpr uint64_t leaf_range_abs = 0xff / 2;
            float sd = signed_distance;
            // scale signed distance to be normalized within truncation distance
            sd = std::clamp(sd * truncation_distance_recip, -1.0f, 1.0f);
            // scale up to fit 8-bit integer, add offset to fit within unsigned
            sd = sd * float(leaf_range_abs) + float(leaf_range_abs);
            // shove the bits into leaf cluster
            _value |= uint8_t(sd) << (8 * child_i);
        }
        void set_leaf_empty(uint8_t child_i) {
            // bits 0xff signify an empty leaf
            _value |= 0xff << (8 * child_i);
        }
        bool is_empty() {
            return _value == std::numeric_limits<uint64_t>::max();
        }

        uint64_t _value = 0;
    };
}