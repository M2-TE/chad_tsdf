#pragma once
#include <limits>
#include <cstdint>
#include <cassert>
#include <algorithm>

namespace chad {
    namespace detail {
        // wrapper for signed distance cluster
        struct TSDFs {
            // set 8 bits to represent signed distance, normalized within truncation distance
            void inline set(uint8_t leaf_i, float signed_distance, float truncation_distance_recip) noexcept {
                // absolute value range for signed distances stored as integers
                static constexpr uint64_t sd_range_abs = std::numeric_limits<uint8_t>::max() / 2;
                
                float sd = signed_distance;
                // scale signed distance to be normalized within truncation distance
                sd = std::clamp(sd * truncation_distance_recip, -1.0f, 1.0f);
                // scale up to fit 8-bit integer, add offset to fit within unsigned
                sd = sd * float(sd_range_abs) + float(sd_range_abs); // range should be [-127, 128]

                // TODO: verify that sd is never 0xff

                // shove the bits into leaf cluster
                _value |= uint64_t(sd) << uint64_t(leaf_i * 8);
            }
            // set 8 bits to represent an empty leaf
            void inline set_empty(uint8_t leaf_i) noexcept {
                // bits 0xff for signed distance signify an empty leaf
                _value |= uint64_t(0xff) << uint64_t(leaf_i * 8);
            }
            // retrieve signed distance from single leaf if it is not empty
            auto inline try_get(uint8_t leaf_i, float truncation_distance) const -> std::pair<float, bool> {
                // absolute value range for signed distances stored as integers
                static constexpr uint64_t sd_range_abs = std::numeric_limits<uint8_t>::max() / 2;

                // get the 8 bits corresponding to the requested leaf
                uint64_t leaf_bits = _value >> uint64_t(leaf_i * 8);
                leaf_bits &= 0xff; // mask out other bits

                // check if the leaf is empty
                if (leaf_bits == 0xff) return { 0.0f, false };

                // offset value to represent [-127, 128]
                float signed_distance = float(leaf_bits) - float(sd_range_abs);
                // scale signed distance back up to normalized [-1, 1]
                signed_distance *= float(1.0 / float(sd_range_abs));
                // scale back to denormalized signed distance
                signed_distance *= truncation_distance;
                return { signed_distance, true };
            }
            uint64_t _value;
        };
        // wrapper for weight cluster
        struct Weights {
            // set 8 bits to represent a single weight
            void inline set(uint8_t leaf_i, uint8_t weight) noexcept {
                uint8_t truncated_weight = std::min<uint8_t>(weight, std::numeric_limits<uint8_t>::max());
                _value |= uint64_t(truncated_weight) << uint64_t(leaf_i * 8);
            }
            void inline set_empty(uint8_t leaf_i) noexcept {
                // bits 0xff for signed distance signify an empty leaf
                _value |= uint64_t(0xff) << uint64_t(leaf_i * 8);
            }
            // auto try_get(uint8_t leaf_i) const -> std::pair<float, bool> {
            // }
            uint64_t _value;
        };
    }

    // cluster of 8 separate 8-bit leaves
    struct LeafCluster {
        LeafCluster(): _value(0) {}
        
        bool is_empty() {
            return _value == std::numeric_limits<uint64_t>::max();
        }

        union {
            uint64_t        _value;
            detail::TSDFs   _tsdfs;
            detail::Weights _weigh;
        };
    };
}