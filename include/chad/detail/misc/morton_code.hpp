#pragma once
#if defined(__BMI2__)
#   include <immintrin.h>
#else
#   error "Requires BMI2 instruction set"
#endif

namespace {
    auto inline pdep(std::uint64_t src, std::uint64_t mask) noexcept -> std::uint64_t {
        return _pdep_u64(src, mask);
    }
    auto inline pext(std::uint64_t src, std::uint64_t mask) noexcept -> std::uint64_t {
        return _pext_u64(src, mask);
    }
}

namespace chad::detail {
    struct MortonCode {
        constexpr MortonCode(std::uint64_t value) noexcept: _value(value) {}
        MortonCode(const glm::aligned_ivec3& vox_pos) noexcept {
            _value = encode(vox_pos);
        }
        MortonCode(const glm::aligned_vec3& point, float sdf_res_reciprocal) noexcept {
            // convert to voxel coordinate and discretize with floor()
            glm::aligned_vec3 point_discretized = glm::floor(point * sdf_res_reciprocal);
            _value = encode(glm::aligned_ivec3{ point_discretized });
        }

        auto inline static encode(const glm::aligned_ivec3& vox_pos) noexcept-> std::uint64_t {
            // truncate from 32-bit int to 21-bit uint
            std::uint32_t x = (1 << 20) + static_cast<std::uint32_t>(vox_pos.x);
            std::uint32_t y = (1 << 20) + static_cast<std::uint32_t>(vox_pos.y);
            std::uint32_t z = (1 << 20) + static_cast<std::uint32_t>(vox_pos.z);
            // parallel bit deposit (pdep)
            return pdep(x, 0x9249249249249249) | pdep(y, 0x2492492492492492) | pdep(z, 0x4924924924924924);
        }
        auto inline decode() const noexcept -> glm::aligned_ivec3 {
            // parallel bit extract (pext), followed by expanding 21-bit uint back to 32-bit int
            std::int32_t x = pext(_value, 0x9249249249249249) - static_cast<std::uint64_t>(1 << 20);
            std::int32_t y = pext(_value, 0x2492492492492492) - static_cast<std::uint64_t>(1 << 20);
            std::int32_t z = pext(_value, 0x4924924924924924) - static_cast<std::uint64_t>(1 << 20);
            return { x, y, z };
        }

        // mask out lower bits of the morton code, as per depth (basically higher discretization)
        template<std::uint64_t DEPTH>
        auto constexpr inline mask() const noexcept -> MortonCode {
            constexpr std::uint64_t shift_distance = 63 - DEPTH * 3;
            constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> shift_distance << shift_distance;
            return _value & mask;
        }
        // mask out lower bits of the morton code, as per depth (basically higher discretization)
        auto constexpr inline mask(std::uint64_t depth) const noexcept -> MortonCode {
            std::uint64_t shift_distance = 63 - depth * 3;
            std::uint64_t mask = static_cast<std::uint64_t>(-1) >> shift_distance << shift_distance;
            return _value & mask;
        }

        // get child index at given depth
        template<std::uint64_t DEPTH, std::uint64_t DEPTH_SPAN>
        auto constexpr inline child() const noexcept -> std::uint64_t {
            // shift relevant bits for current depth to LSB
            constexpr std::uint64_t shift = (21 - DEPTH - DEPTH_SPAN) * 3;
            // mask out all the other bits (will be inverted in next step)
            constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> DEPTH_SPAN * 3 << DEPTH_SPAN * 3;
            return _value >> shift & ~mask;
        }
        // get child index at given depth
        template<std::uint64_t DEPTH_SPAN>
        auto constexpr inline child(std::uint64_t depth) const noexcept -> std::uint64_t {
            // shift relevant bits for current depth to LSB
            std::uint64_t shift = (21 - depth - DEPTH_SPAN) * 3;
            // mask out all the other bits (will be inverted in next step)
            constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> DEPTH_SPAN * 3 << DEPTH_SPAN * 3;
            return _value >> shift & ~mask;
        }

        auto constexpr inline operator==(MortonCode other) const noexcept -> bool {
            return _value == other._value;
        }
        auto constexpr inline operator<(MortonCode other) const noexcept -> bool {
            return _value < other._value;
        }
        auto constexpr inline operator>(MortonCode other) const noexcept -> bool {
            return _value > other._value;
        }
        auto constexpr inline operator&(MortonCode other) const noexcept -> MortonCode {
            return _value & other._value;
        }
        auto constexpr inline operator>>(std::uint64_t shift) const noexcept -> MortonCode {
            return _value >> shift;
        }
        auto constexpr inline operator<<(std::uint64_t shift) const noexcept -> MortonCode {
            return _value << shift;
        }
        auto constexpr inline friend operator==(std::uint64_t lhs, MortonCode rhs) noexcept -> bool {
            return lhs == rhs._value;
        }
        auto constexpr inline friend operator<(std::uint64_t lhs, MortonCode rhs) noexcept -> bool {
            return lhs < rhs._value;
        }
        auto constexpr inline friend operator>(std::uint64_t lhs, MortonCode rhs) noexcept -> bool {
            return lhs > rhs._value;
        }
        auto constexpr inline friend operator&(std::uint64_t lhs, MortonCode rhs) noexcept -> MortonCode {
            return lhs & rhs._value;
        }

        std::uint64_t _value;
    };
}

// specialize the std::hash operator() for MortonCode
namespace std {
    template<>
    struct hash<chad::detail::MortonCode> {
        size_t inline operator()(const chad::detail::MortonCode& mc) const {
            return mc._value;
        }
    };
}

// specialize the fmt::formatter for MortonCode
namespace fmt {
    template<>
    struct formatter<chad::detail::MortonCode>: formatter<std::string> {
        auto format(const chad::detail::MortonCode& mc, format_context& ctx) const -> format_context::iterator {
            std::string str_raw = std::bitset<63>(mc._value).to_string();
            std::string str_fin;
            str_fin.reserve(63 + 63/3);
            for (auto it = str_raw.cbegin(); it < str_raw.cend(); it += 3) {
                for (std::uint8_t i = 0; i < 3; i++) {
                    str_fin.push_back(*(it + i));
                }
                str_fin.push_back('\'');
            }
            str_fin.back() = ' ';
            return formatter<std::string>::format(str_fin, ctx);
        }
    };
}
