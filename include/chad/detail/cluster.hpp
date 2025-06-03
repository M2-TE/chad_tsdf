#pragma once
#include <limits>
#include <cstdint>
#include <algorithm>

namespace chad {
    // cluster of 8 separate 8-bit leaves
    struct LeafCluster {
        void set_leaf_sd_empty(uint8_t child_i) {
            // bits 0xff for signed distance signify an empty leaf
            _value |= 0xff << (8 * child_i);
        }
        void set_leaf_sd(uint8_t child_i, float signed_distance, float truncation_distance_recip) {
            // absolute value range for signed distances stored as integers
            static constexpr uint64_t sd_range_abs = std::numeric_limits<uint8_t>::max() / 2;
            float sd = signed_distance;
            // scale signed distance to be normalized within truncation distance
            sd = std::clamp(sd * truncation_distance_recip, -1.0f, 1.0f);
            // scale up to fit 8-bit integer, add offset to fit within unsigned
            sd = sd * float(sd_range_abs) + float(sd_range_abs);
            // shove the bits into leaf cluster
            _value |= uint8_t(sd) << (8 * child_i);
        }
        void set_leaf_weight(uint8_t child_i, uint8_t weight) {
            _value |= weight << (8 * child_i);
        }
        bool is_empty() {
            return _value == std::numeric_limits<uint64_t>::max();
        }

        uint64_t _value = 0;
    };
}