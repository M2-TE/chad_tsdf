#pragma once
#include <functional>
#if !defined(__BMI2__)
#   error "Requires BMI2 instruction set"
#endif
#include <glm/glm.hpp>
#include <libmorton/morton.h>

namespace chad::detail {
    struct MortonCode {
        MortonCode(uint64_t value): _value(value) {}
        MortonCode(glm::ivec3 vox_pos) noexcept {
            // truncate from 32-bit int to 21-bit int
            uint32_t x, y, z;
            x = (1 << 20) + uint32_t(vox_pos.x);
            y = (1 << 20) + uint32_t(vox_pos.y);
            z = (1 << 20) + uint32_t(vox_pos.z);
            _value = uint64_t(libmorton::morton3D_64_encode(x, y, z));
        }
        auto inline decode() const noexcept -> glm::ivec3 {
            uint_fast32_t x, y, z;
            libmorton::morton3D_64_decode(_value, x, y, z);
            // expand from 21-bit uint back to 32-bit int
            x -= 1 << 20;
            y -= 1 << 20;
            z -= 1 << 20;
            return { int32_t(x), int32_t(y), int32_t(z) };
        }
        bool inline operator==(const MortonCode& other) const noexcept {
            return _value == other._value;
        }

        uint64_t _value;
    };

    using MortonVector = std::vector<std::pair<glm::vec3, MortonCode>>;
    auto calc_morton_vector(const std::vector<std::array<float, 3>>& points, const float sdf_res) -> MortonVector;
    auto sort_morton_vector(MortonVector& points_mc) -> std::vector<glm::vec3>;
}

namespace std {
    template<> struct hash<chad::detail::MortonCode> {
        size_t inline operator()(const chad::detail::MortonCode& mc) const noexcept {
            return mc._value;
        }
    };
}