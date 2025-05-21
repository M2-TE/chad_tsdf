#pragma once
#if !defined(__BMI2__)
#   error "Requires BMI2 instruction set"
#endif
#include <libmorton/morton.h>
#include <gtl/phmap.hpp>
#include <glm/glm.hpp>

namespace chad {
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
}

namespace std {
    template<> struct hash<chad::MortonCode> {
        size_t inline operator()(const chad::MortonCode& mc) const noexcept {
            return mc._value;
        }
    };
}