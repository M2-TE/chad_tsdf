#pragma once
#include <limits>
#include <cstdint>
#include <cassert>
#include <algorithm>

// TODO: move header out of detail namespace

namespace chad {
    // cluster of 8 separate 8-bit leaves
    struct LeafCluster {
        LeafCluster(): _value(0) {}
        void set_leaf_sd_empty(uint64_t leaf_i) {
            // bits 0xff for signed distance signify an empty leaf
            _value |= uint64_t(0xff) << uint64_t(8 * leaf_i);
        }
        void set_leaf_sd(uint8_t leaf_i, float signed_distance, float truncation_distance_recip) {
            // absolute value range for signed distances stored as integers
            static constexpr uint64_t sd_range_abs = std::numeric_limits<uint8_t>::max() / 2;
            float sd = signed_distance;
            // scale signed distance to be normalized within truncation distance
            sd = std::clamp(sd * truncation_distance_recip, -1.0f, 1.0f);
            // scale up to fit 8-bit integer, add offset to fit within unsigned
            sd = sd * float(sd_range_abs) + float(sd_range_abs); // range should be [-127, 128]

            // shove the bits into leaf cluster
            _value |= uint64_t(sd) << uint64_t(8 * leaf_i);
        }
        auto try_get_leaf_sd(uint8_t leaf_i, float truncation_distance) -> std::pair<float, bool> {
            // get the 8 bits corresponding to the requested leaf
            uint64_t leaf_bits = _value >> (leaf_i * 8);
            leaf_bits &= 0xff; // mask out other bits

            // check if the leaf is empty
            if (leaf_bits == 0xff) return { 0.0f, false };

            // offset value to represent [-127, 128]
            float signed_distance = float(leaf_bits) - 127.0f;
            // scale signed distance back up to normalized [-1, 1]
            signed_distance *= float(1.0 / 127.0);
            // scale back to denormalized signed distance
            signed_distance *= truncation_distance;
            return { signed_distance, true };
        }

        void set_leaf_weight(uint8_t leaf_i, uint32_t weight) {
            uint32_t truncated_weight = std::min<uint32_t>(weight, std::numeric_limits<uint32_t>::max());
            _value |= uint64_t(truncated_weight) << uint64_t(8 * leaf_i);
        }
        // auto try_get_leaf_weight(uint8_t leaf_i) -> std::pair<float, bool> {

        // }
        
        bool is_empty() {
            return _value == std::numeric_limits<uint64_t>::max();
        }

        uint64_t _value;
    };
}