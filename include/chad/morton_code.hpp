#pragma once
#if !defined(__BMI2__)
#   error "Requires BMI2 instruction set"
#endif
#include <libmorton/morton.h>
#include <glm/glm.hpp>

struct MortonCode {
    MortonCode(uint64_t value): _value(value) {}
    MortonCode(glm::ivec3 vox_pos): _value(encode(vox_pos)) {
    }
    auto inline decode() -> glm::ivec3 {
        return decode(_value);
    }

    auto static inline encode(glm::ivec3 vox_pos) -> uint64_t {
        // truncate from 32-bit int to 21-bit int
        uint32_t x, y, z;
        x = (1 << 20) + uint32_t(vox_pos.x);
        y = (1 << 20) + uint32_t(vox_pos.y);
        z = (1 << 20) + uint32_t(vox_pos.z);
        return libmorton::morton3D_64_encode(x, y, z);
    }
    auto static inline decode(uint64_t code) -> glm::ivec3 {
        uint_fast32_t x, y, z;
        libmorton::morton3D_64_decode(code, x, y, z);
        // expand from 21-bit uint back to 32-bit int
        x -= 1 << 20;
        y -= 1 << 20;
        z -= 1 << 20;
        return { int32_t(x), int32_t(y), int32_t(z) };
    }
    uint64_t _value;
};