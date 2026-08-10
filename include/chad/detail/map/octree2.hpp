#pragma once
#include "chad/detail/misc/morton_code.hpp"

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

        void inline clear() {
            _nodes.clear();
            _roots.clear();
        }
        void inline insert(MortonCode morton_code, float signed_distance) {
            // mask out the bits relevant for hashmap lookup
            constexpr std::uint64_t shift_distance = 63 - DEPTH_START * 3;
            constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> shift_distance << shift_distance;

            // obtain node using masked morton code as the key
            auto [it, emplaced_b] = _roots.try_emplace(morton_code & mask);
            // if one didn't exist yet, create it
            if (emplaced_b) {
                it->second = _nodes.size();
                _nodes.emplace_back();
            }
            NodeAddr node_addr = it->second;

            // walk through each node to reach leaves
            for (std::uint64_t depth = DEPTH_START; depth < 21 - DEPTH_SPAN; depth += DEPTH_SPAN) {
                // walk to next child node
                std::uint64_t child_index = morton_code.child<DEPTH_SPAN>(depth);
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
            std::uint64_t leaf_index = morton_code.child<21 - DEPTH_SPAN, DEPTH_SPAN>();
            Leaf& leaf = _nodes[node_addr]._leaves[leaf_index];

            // weighted merge of signed distance and weight increment
            leaf._signed_distance = leaf._signed_distance * static_cast<float>(leaf._weight) + signed_distance;
            leaf._weight++;
            leaf._signed_distance = leaf._signed_distance / static_cast<float>(leaf._weight);
        }

        std::vector<Node> _nodes;
        gtl::parallel_flat_hash_map<MortonCode, NodeAddr> _roots; // tree begins at DEPTH_START
        static constexpr std::uint64_t _DEPTH_START = DEPTH_START;
        static constexpr std::uint64_t _DEPTH_SPAN = DEPTH_SPAN;
    };
}
