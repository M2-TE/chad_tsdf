#pragma once
#include "chad/detail/morton_code.hpp"

namespace chad::detail::mapping {
    struct Octree2 {
        using NodeAddr = uint32_t;
        struct Leaf {
            float _signed_distance = 0.0f;
            uint32_t _weight = 0;
        };
        using Node = std::array<Leaf, 8*8*8>; // this node spans 3 depths

        void clear() {
            _nodes.clear();
        }

        gtl::parallel_node_hash_map<MortonCode, Node> _nodes; // starts at depth 18? 17?
    };
}
