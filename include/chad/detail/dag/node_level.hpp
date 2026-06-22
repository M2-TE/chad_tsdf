#pragma once
#include "chad/detail/dag/node.hpp"
#include "chad/detail/virtual_array.hpp"

namespace chad::detail::dag {
    struct NodeLevel {
        struct FncHash {
            FncHash(const VirtualArray<NodeSegment>& segments): _segments(segments) {}
            auto inline operator()(ADDR_T addr) const noexcept -> std::uint64_t {
                // read node header
                NodeHead head = _segments[addr].head;
                std::size_t child_count = std::popcount(head._child_mask);
                // hash entire node
                std::uint64_t hash = 0;
                for (std::size_t i = 1; i <= child_count; i++) {
                    ADDR_T child_addr = _segments[addr + i].child_addr;
                    hash = gtl::HashState::combine(hash, child_addr);
                }
                return hash;
            }
            const VirtualArray<NodeSegment>& _segments;
        };
        struct FncEq {
            FncEq(const VirtualArray<NodeSegment>& segments): _segments(segments) {}
            auto inline operator()(ADDR_T addr_a, ADDR_T addr_b) const noexcept -> bool {
                // compare child masks
                std::uint8_t mask_a = _segments[addr_a].head._child_mask;
                if (mask_a != _segments[addr_b].head._child_mask) {
                    return false;
                }
                else {
                    // count children (only need the count for a)
                    std::uint8_t child_count_a = std::popcount(mask_a);

                    // compare entire nodes
                    int cmp = std::memcmp(
                        _segments.data() + addr_a + 1,
                        _segments.data() + addr_b + 1,
                        child_count_a * sizeof(ADDR_T));
                    return cmp == 0;
                }
            }
            const VirtualArray<NodeSegment>& _segments;
        };

        NodeLevel(): _addr_set(0, FncHash(_segments), FncEq(_segments)) {
            // reserve first index
            _segments.push_back(NodeSegment{});
            _occupied_segments_n = _segments.size();
        }

        std::uint32_t _uniques_n, _dupes_n; // for statistics
        std::uint32_t _occupied_segments_n; // number of actually occupied segments
        VirtualArray<NodeSegment> _segments; // raw unaligned node data, including potentially unused segments near the end
        gtl::parallel_flat_hash_set<ADDR_T, FncHash, FncEq> _addr_set; // set of addresses
    };
}
