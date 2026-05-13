#pragma once
// #include "chad/cluster.hpp"
// #include "chad/detail/morton.hpp"
#include "chad/detail/virtual_array.hpp"

// TODO
namespace chad::detail::dag {

    // TODO: templated NodeAddress with depth as the template for more type safety?
    // template<std::size_t DEPTH>
    // struct NodeAddress {
    //     uint32_t node_address;
    // };
    struct NodeAddress {
        uint32_t _address;
        uint32_t _depth;
    };

    template<std::size_t LEVELS = 21>
    struct Storage {
        static constexpr std::size_t MAX_DEPTH = LEVELS - 1;
        // // 20 levels of standard nodes
        // std::array<NodeLevel, MAX_DEPTH> _node_levels;
        // // 1 level of leaf clusters
        // LeafClusterLevel _leaf_clusters;
    };
};