#pragma once
#include "chad/detail/morton_code.hpp"

namespace chad::detail::map {
    // DEPTH_CACHE (default 17) -> at which depth nodes are cached
    // DEPTH_SPAN  (default  2) -> how many depths a single node spans
    template<std::uint64_t DEPTH_CACHE = 17, std::uint64_t DEPTH_SPAN = 2>
    struct Octree3 {
        static_assert((21 - DEPTH_CACHE) % DEPTH_SPAN == 0);
        using NodeAddr = std::uint32_t;
        struct Leaf {
            float _signed_distance;
            std::uint32_t _weight;
        };
        struct Node {
            constexpr static std::uint64_t DEPTH_CHILDREN = 0b1 << DEPTH_SPAN << DEPTH_SPAN << DEPTH_SPAN;
            union {
                std::array<Leaf,  DEPTH_CHILDREN> _leaves;
                std::array<NodeAddr, DEPTH_CHILDREN> _children;
            };
        };

        void inline clear() {
            _roots.clear();
            _nodes.clear();
        }
        void inline insert(MortonCode morton_code, float signed_distance) {

        }

        std::vector<Node> _nodes;
        gtl::parallel_flat_hash_map<MortonCode, NodeAddr> _roots; // tree begins at DEPTH_START
    };
}
