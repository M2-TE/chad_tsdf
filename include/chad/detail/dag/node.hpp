#pragma once

namespace chad::detail::dag {
    using ADDR_T = std::uint32_t;
    static_assert(std::is_unsigned_v<ADDR_T>);
    static_assert(std::is_integral_v<ADDR_T>);

    struct alignas(ADDR_T) NodeHead {
        uint8_t  _child_mask;
        uint8_t  _depth;
        uint16_t _ref_count;
    };
    static_assert(sizeof(NodeHead) == sizeof(ADDR_T));

    union alignas(ADDR_T) NodeSegment {
        NodeHead head;
        ADDR_T child_addr;
    };
    static_assert(sizeof(NodeSegment) == sizeof(ADDR_T));
}
