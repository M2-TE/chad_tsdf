#pragma once
#include "chad/detail/morton_code.hpp"

namespace chad::detail::map {
    // DEPTH_START (default 17) -> at which depth root nodes will start
    // DEPTH_SPAN  (default  2) -> how many depths a single node spans
    template<std::uint64_t DEPTH_START = 17, std::uint64_t DEPTH_SPAN = 2>
    struct Octree2 {
        static_assert((21 - DEPTH_START) % DEPTH_SPAN == 0);
        using NodeAddr = std::uint32_t;
        struct Leaf {
            float _signed_distance;
            std::uint32_t _weight;
        };
        struct Node {
            constexpr static std::uint64_t DEPTH_CHILDREN = 0b1 << DEPTH_SPAN << DEPTH_SPAN << DEPTH_SPAN;
            union {
                std::array<Leaf, DEPTH_CHILDREN> _leaves;
                std::array<NodeAddr, DEPTH_CHILDREN> _children;
            };
        };

        auto static consteval get_start() -> std::uint64_t {
            return DEPTH_START;
        }
        auto static consteval get_span() -> std::uint64_t {
            return DEPTH_SPAN;
        }
        void inline clear() {
            _roots.clear();
            _nodes.clear();
        }
        void inline insert(MortonCode morton_code, float signed_distance) {
            // mask out the bits relevant for hashmap lookup
            constexpr std::uint64_t DEPTH_BITS = 3; // every child in a single depth (2x2x2) is addressable by 3 bits
            constexpr std::uint64_t shift_distance = sizeof(std::uint64_t) * 8 - DEPTH_START * DEPTH_BITS - 1;
            constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> shift_distance << shift_distance;

            // obtain node using masked morton code as the key
            auto [it, emplaced_b] = _roots.try_emplace(morton_code & mask);
            // if one didn't exist yet, create it
            if (emplaced_b) {
                it->second = _nodes.size();
                _nodes.emplace_back();
            }
            NodeAddr node_addr = it->second;

            // lambda to obtain child index from a morton code for a given node at a given depth
            auto obtain_child_index = [](MortonCode morton_code, std::uint64_t depth) -> std::uint64_t {
                // shift relevant bits for current depth to LSB
                std::uint64_t shift = (21 - depth - DEPTH_SPAN) * DEPTH_BITS;
                // mask out all the other bits (will be inverted in next step)
                constexpr std::uint64_t mask_shift = DEPTH_SPAN * DEPTH_BITS;
                constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> mask_shift << mask_shift;
                // perform shift and mask on given morton code
                return (morton_code >> shift & ~mask)._value;
            };

            auto obtain_leaf_index = [](MortonCode morton_code) -> std::uint64_t {
                // shift relevant bits for current depth to LSB
                constexpr std::uint64_t shift = (21 - (21 - DEPTH_SPAN) - DEPTH_SPAN) * DEPTH_BITS; // TODO: untangle this mess
                // mask out all the other bits (will be inverted in next step)
                constexpr std::uint64_t mask_shift = DEPTH_SPAN * DEPTH_BITS;
                constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> mask_shift << mask_shift;
                // perform shift and mask on given morton code
                return (morton_code >> shift & ~mask)._value;
            };

            // walk through each node to reach leaves
            for (std::uint64_t depth = DEPTH_START; depth < 21 - DEPTH_SPAN; depth += DEPTH_SPAN) {
                // walk to next child node
                std::uint64_t child_index = obtain_child_index(morton_code, depth);
                NodeAddr child_addr = _nodes[node_addr]._children[child_index];
                // if one didn't exist yet, create it
                if (child_addr == 0) {
                    child_addr = _nodes.size();
                    _nodes[node_addr]._children[child_index] = _nodes.size();
                    _nodes.emplace_back();
                }
                node_addr = child_addr;
            }

            // walk to the leaf node
            std::uint64_t leaf_index = obtain_leaf_index(morton_code);
            Leaf& leaf = _nodes[node_addr]._leaves[leaf_index];

            // weighted merge of signed distance and weight increment
            leaf._signed_distance = leaf._signed_distance * static_cast<float>(leaf._weight) + signed_distance;
            leaf._weight++;
            leaf._signed_distance = leaf._signed_distance / static_cast<float>(leaf._weight);
        }

        std::vector<Node> _nodes;
        gtl::parallel_flat_hash_map<MortonCode, NodeAddr> _roots; // tree begins at DEPTH_START
    };
}
