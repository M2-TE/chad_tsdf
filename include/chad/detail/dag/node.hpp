#pragma once

namespace chad::detail::dag {
    using ADDR_T = std::uint32_t;
    static_assert(std::is_unsigned_v<ADDR_T>);
    static_assert(std::is_integral_v<ADDR_T>);

    struct Addresses {
        dag::ADDR_T _tsdfs;
        dag::ADDR_T _weigh;
    };

    // the header of every node to store metadata
    struct alignas(ADDR_T) NodeHead {
        std::uint8_t  _child_mask;
        std::uint8_t  _depth; // helps debugging (could be replaced)
        std::uint16_t _ref_count;
    };
    static_assert(sizeof(NodeHead) == sizeof(ADDR_T));

    // node segments are the raw data that DAGs store, containing either a header or body segment
    union alignas(ADDR_T) NodeSegment {
        NodeHead _head;
        ADDR_T _child_addr;
    };
    static_assert(sizeof(NodeSegment) == sizeof(ADDR_T));
}
