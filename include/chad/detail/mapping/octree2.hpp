#pragma once
#include "chad/detail/morton_code.hpp"

namespace {
    // calculate the number of depths spanned by a cluster of nodes, each containing NODE_CHILDREN children
    template<std::size_t NODE_CHILDREN>
    auto constexpr calc_spanned_depth(std::size_t depth_nodes_n, std::size_t depth = 0) -> std::size_t {
        if (depth_nodes_n == 1) return depth;
        else return calc_spanned_depth<NODE_CHILDREN>(depth_nodes_n / NODE_CHILDREN, depth + 1);
    }
}

namespace chad::detail::mapping {
    struct Octree2 {
        using NodeAddr = std::uint32_t;
        struct Leaf {
            float _signed_distance = 0.0f;
            std::uint32_t _weight = 0;
        };
        struct Node {
            // DEPTH_SPAN depths of 2x2x2 nodes each
            std::array<Leaf, 8*8*8> _leaves;

            // validation
            constexpr static std::size_t DEPTH_SPAN = calc_spanned_depth<8>(sizeof(_leaves) / sizeof(Leaf));
        };

        void inline clear() {
            _nodes.clear();
        }
        void inline insert(MortonCode morton_code, float signed_distance) {
            // mask out the bits relevant for hashmap lookup
            constexpr std::uint64_t shift_distance = sizeof(std::uint64_t) * 8 - DEPTH_START * 3 - 1;
            constexpr std::uint64_t mask = std::uint64_t(-1) >> shift_distance << shift_distance;

            // obtain node using masked morton code as the key
            auto [it, emplaced_b] = _nodes.try_emplace(morton_code & mask);

            // obtain leaf index by inverting the mask
            Node& node = it->second;
            Leaf& leaf = node._leaves[morton_code._value & ~mask];

            // weighted merge of signed distance and weight increment
            leaf._signed_distance = leaf._signed_distance * static_cast<float>(leaf._weight) + signed_distance;
            leaf._weight++;
            leaf._signed_distance = leaf._signed_distance / static_cast<float>(leaf._weight);
        }

        gtl::parallel_node_hash_map<MortonCode, Node> _nodes; // starts at depth DEPTH_START

        // validation
        constexpr static std::uint64_t DEPTH_START = 21 - Node::DEPTH_SPAN;
    };
}
