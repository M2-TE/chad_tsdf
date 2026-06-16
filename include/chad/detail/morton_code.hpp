#pragma once
#if !defined(__BMI2__)
#   error "Requires BMI2 instruction set"
#endif

namespace chad::detail {
    struct MortonCode {
        constexpr MortonCode(std::uint64_t value): _value(value) {}
        constexpr MortonCode(const glm::aligned_ivec3& vox_pos) {
            encode(vox_pos);
        }
        constexpr MortonCode(const glm::aligned_vec3& point, float sdf_res_reciprocal) {
            // convert to voxel coordinate and discretize with floor()
            glm::aligned_vec3 point_discretized = glm::floor(point * sdf_res_reciprocal);
            encode(glm::aligned_ivec3{ point_discretized });
        }
        // TODO: remove this once revamp is done
        [[deprecated]]
        constexpr MortonCode(const glm::ivec3& vox_pos) {
            encode(vox_pos);
        }

        void inline encode(const glm::aligned_ivec3& vox_pos) {
            // truncate from 32-bit int to 21-bit int
            std::uint32_t x, y, z;
            x = (1 << 20) + static_cast<std::uint32_t>(vox_pos.x);
            y = (1 << 20) + static_cast<std::uint32_t>(vox_pos.y);
            z = (1 << 20) + static_cast<std::uint32_t>(vox_pos.z);
            _value = static_cast<std::uint64_t>(libmorton::morton3D_64_encode(x, y, z));
        }
        auto inline decode() const -> glm::aligned_ivec3 {
            uint_fast32_t x, y, z;
            libmorton::morton3D_64_decode(_value, x, y, z);
            // expand from 21-bit uint back to 32-bit int
            x -= 1 << 20;
            y -= 1 << 20;
            z -= 1 << 20;
            return { static_cast<std::int32_t>(x), static_cast<std::int32_t>(y), static_cast<std::int32_t>(z) };
        }

        auto constexpr inline operator==(MortonCode other) const -> bool {
            return _value == other._value;
        }
        auto constexpr inline operator<(MortonCode other) const -> bool {
            return _value < other._value;
        }
        auto constexpr inline operator>(MortonCode other) const -> bool {
            return _value > other._value;
        }
        auto constexpr inline operator&(MortonCode other) const -> MortonCode {
            return _value & other._value;
        }
        auto constexpr inline operator>>(std::uint64_t shift) const -> MortonCode {
            return _value >> shift;
        }
        auto constexpr inline operator<<(std::uint64_t shift) const -> MortonCode {
            return _value << shift;
        }

        auto constexpr inline friend operator==(std::uint64_t lhs, MortonCode rhs) -> bool {
            return lhs == rhs._value;
        }
        auto constexpr inline friend operator<(std::uint64_t lhs, MortonCode rhs) -> bool {
            return lhs < rhs._value;
        }
        auto constexpr inline friend operator>(std::uint64_t lhs, MortonCode rhs) -> bool {
            return lhs > rhs._value;
        }
        auto constexpr inline friend operator&(std::uint64_t lhs, MortonCode rhs) -> MortonCode {
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
            std::string str = std::bitset<63>(mc._value).to_string();
            return formatter<std::string>::format(str, ctx);
        }
    };
}
