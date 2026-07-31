#pragma once
#include "chad/detail/dag/node_level.hpp"
#include "chad/detail/dag/leaf_cluster_level.hpp"

namespace chad::detail::dag {
    struct Storage {
        // add a DAG node and return address of new or existing one
        auto inline add_node(const std::array<ADDR_T, 8>& children, std::uint32_t depth) -> ADDR_T {
            auto& level = _node_levels[depth];
            // check if theres enough space for a full placeholder node
            if (level._occupied_segments_n + 9 >= level._segments.size()) {
                level._segments.resize(level._segments.size() + 9);
            }

            // write placeholder node into raw data vector
            ADDR_T placeholder_addr = level._occupied_segments_n;
            NodeSegment* placeholder_p = level._segments.data() + placeholder_addr;
            placeholder_p->head._child_mask = 0;
            placeholder_p->head._depth = depth;
            placeholder_p->head._ref_count = 1;

            // add only valid children to save space
            std::uint8_t children_n = 0;
            for (std::uint8_t child_i = 0; child_i < 8; child_i++) {
                if (children[child_i] == 0) continue;
                placeholder_p[children_n].child_addr = children[child_i];
                placeholder_p->head._child_mask |= 1 << child_i;
                children_n++;
            }

            // emplace placeholder node if it's a new one
            auto [old_addr_it, new_addr_b] = level._addr_set.emplace(placeholder_addr);
            if (new_addr_b) {
                level._uniques_n++;
                level._occupied_segments_n += children_n + 1;
                return placeholder_addr;
            }
            else {
                ADDR_T old_addr = *old_addr_it;
                level._segments[old_addr].head._ref_count++;
                level._dupes_n++;
                return old_addr;
            }
        }

        // add a DAG leaf cluster and return address of new or existing one
        auto inline add_lc(LeafCluster lc) -> ADDR_T {
            // append a placeholder node (will only ever be 1 placeholder in this vector, hence push_back() and back())
            ADDR_T new_addr = _leaf_cluster_level._uniques_n + 1;
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
        // for synchronization during async operations
        std::mutex _mutex;
    };
};
