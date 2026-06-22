#pragma once
#include "chad/detail/dag/node_level.hpp"
#include "chad/detail/dag/leaf_cluster_level.hpp"

namespace chad::detail::dag {
    struct Storage {
        // 20 levels of standard nodes
        std::array<NodeLevel, 20> _node_levels;
        // 1 level of leaf clusters
        LeafClusterLevel _leaf_cluster_level;
        // for synchronization during async map optimizations
        std::mutex _mutex;
    };
};
