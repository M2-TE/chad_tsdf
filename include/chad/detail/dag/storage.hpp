#pragma once
#include "chad/detail/dag/node_level.hpp"
#include "chad/detail/misc/morton_code.hpp"
#include "chad/detail/dag/leaf_cluster_level.hpp"

namespace chad::detail::dag {
    struct Storage {
        void clear() {
            for (auto& level: _node_levels) level.clear();
            _leaf_cluster_level.clear();
        }

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
            placeholder_p->_head._child_mask = 0;
            placeholder_p->_head._depth = depth;
            placeholder_p->_head._ref_count = 1;

            // add only valid children to save space
            std::uint8_t children_n = 0;
            for (std::uint8_t child_i = 0; child_i < 8; child_i++) {
                if (children[child_i] == 0) continue;
                // append valid child to placeholder node
                placeholder_p[children_n + 1]._child_addr = children[child_i];
                placeholder_p->_head._child_mask |= 1 << child_i;
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
                level._segments[old_addr]._head._ref_count++;
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

        // get child address of given node; returns 0 if none is found
        auto inline get_node(std::uint32_t parent_depth, ADDR_T parent_addr, std::uint8_t child_i) const -> ADDR_T {
            // fetch node data
            NodeSegment parent_segment = _node_levels[parent_depth]._segments[parent_addr];

            // check if the child exists
            std::uint8_t child_bit = 1 << child_i;
            if (parent_segment._head._child_mask & child_bit) {
                // count the number of children that are stored before this one
                std::uint8_t masked = parent_segment._head._child_mask & (child_bit - 1);
                std::uint8_t child_count = std::popcount(masked);
                // child count will correspond to the requested child's index + 1 (accounting for child mask index)
                ADDR_T child_segment_addr = parent_addr + static_cast<ADDR_T>(child_count + 1);
                ADDR_T child_addr = _node_levels[parent_depth]._segments[child_segment_addr]._child_addr;
                return child_addr;
            }
            else return 0;
        }
        // get leaf cluster via its address
        auto inline get_lc(ADDR_T lc_addr) const -> LeafCluster {
            return _leaf_cluster_level._leaf_clusters[lc_addr];
        }
        // get leaf cluster via MortonCode index
        auto inline get_lc(ADDR_T root_addr, MortonCode mc) const -> LeafCluster {
            ADDR_T node_addr = root_addr;
            for (std::uint32_t depth = 0; depth < MAX_DEPTH; depth++) {
                std::uint8_t child_i = (mc._value >> (20 - depth) * 3) & 0b111;

                if (depth < MAX_DEPTH - 1) {
                    node_addr = get_node(depth, node_addr, child_i);
                    if (node_addr == 0) return {};
                    continue;
                }
                else {
                    return get_lc(node_addr);
                }
            }
            // std::unreachable();
            return {};
        }

        // return single tsdf leaf from morton code index
        auto inline get_tsdf_leaf(ADDR_T root_addr, MortonCode mc, float sdf_trunc) const -> std::pair<float, bool> {
            LeafCluster lc = get_lc(root_addr, mc);
            return lc._tsdfs.try_get(child_i, sdf_trunc);
        }

        static constexpr std::uint64_t MAX_DEPTH = 21;
        // 20 levels of standard nodes
        std::array<NodeLevel, MAX_DEPTH - 1> _node_levels;
        // 1 level of leaf clusters
        LeafClusterLevel _leaf_cluster_level;
        // for synchronization during async operations
        std::mutex _mutex;
    };
};
