#pragma once
#include "chad/detail/dag/node_level.hpp"
#include "chad/detail/dag/leaf_cluster_level.hpp"

namespace chad::detail::dag {
    struct Storage {
        // add a DAG leaf cluster and return address of new or existing one
        auto inline add_lc(LeafCluster lc) -> uint32_t {
            // append a placeholder node (will only ever be 1 placeholder in this vector, hence push_back() and back())
            uint32_t new_addr = _leaf_cluster_level._uniques_n + 1;
            auto& leaf_clusters = _leaf_cluster_level._leaf_clusters;
            if (leaf_clusters.size() <= new_addr) leaf_clusters.push_back(lc);
            else                                  leaf_clusters.back() = lc;

            // emplace placeholder node if it's a new one
            auto [old_addr_it, new_addr_b] = _leaf_cluster_level._addr_set.emplace(new_addr);
            if (new_addr_b) {
                _leaf_cluster_level._uniques_n++;
                return new_addr;
            }
            else {
                _leaf_cluster_level._dupes_n++;
                return *old_addr_it;
            }
        }

        // 20 levels of standard nodes
        std::array<NodeLevel, 20> _node_levels;
        // 1 level of leaf clusters
        LeafClusterLevel _leaf_cluster_level;
        // for synchronization during async map optimizations
        std::mutex _mutex;
    };
};
