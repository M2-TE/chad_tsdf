#pragma once
#include <vector>
#include <cstdint>
#include <glm/vec3.hpp>
#include "chad/detail/octree.hpp"
#include "chad/detail/levels.hpp"

namespace chad::detail {
    struct Submap {
        void finalize(const Octree& octree, detail::NodeLevels& node_levels, float sdf_trunc) {
            using namespace detail;
            auto beg = std::chrono::high_resolution_clock::now();

            // trackers for the traversed path and nodes
            std::array<uint8_t,  NodeLevels::MAX_DEPTH> path;
            std::array<uint32_t, NodeLevels::MAX_DEPTH> nodes_oct;
            std::array<std::array<uint32_t, 8>, NodeLevels::MAX_DEPTH> nodes_tsdf;
            std::array<std::array<uint32_t, 8>, NodeLevels::MAX_DEPTH> nodes_weight;
            path.fill(0);
            nodes_oct.fill(0);
            nodes_oct[0] = octree.get_root();
            nodes_tsdf.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
            nodes_weight.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
            const float sdf_trunc_recip = 1.0f / sdf_trunc;

            uint32_t depth = 0;
            while (true) {
                uint8_t child_i = path[depth]++;

                // when all children at this depth were iterated
                if (child_i >= 8) {
                    // create/get nodes from current node level
                    auto& node_level = node_levels._nodes[depth];
                    uint32_t addr_tsdf   = node_level.add(nodes_tsdf  [depth]);
                    uint32_t addr_weight = node_level.add(nodes_weight[depth]);

                    // reset node tracker for handled nodes
                    nodes_tsdf  [depth].fill(0);
                    nodes_weight[depth].fill(0);

                    // check if it's the root node
                    if (depth == 0) {
                        root_addr_tsdf = addr_tsdf;
                        root_addr_weight = addr_weight;
                        break;
                    }
                    else {
                        
                        // continue at parent depth
                        depth--;
                        // created nodes are standard tree nodes
                        uint32_t index_in_parent = path[depth] - 1;
                        nodes_tsdf  [depth][index_in_parent] = addr_tsdf;
                        nodes_weight[depth][index_in_parent] = addr_weight;


                        // fmt::println("depth {}: index in parent: {} addr: {}", depth + 1, index_in_parent, addr_tsdf);
                        // static int COUNTER = 0;
                        // if (COUNTER++ >= 20) exit(0);
                    }
                }
                // node contains node children
                else if (depth < NodeLevels::MAX_DEPTH - 1) {
                    // retrieve child address
                    uint32_t child_addr = octree.get_child_addr(nodes_oct[depth], child_i);
                    if (child_addr == 0) continue;

                    // walk deeper
                    depth++;
                    path[depth] = 0;
                    nodes_oct[depth] = child_addr;
                }
                // node contains leaf children
                else {
                    // retrieve address of current child node
                    uint32_t child_addr = octree.get_child_addr(nodes_oct[depth], child_i);
                    if (child_addr == 0) continue;

                    // retrieve node
                    const Octree::Node& node = octree.get_node(child_addr);
                    
                    // create leaf cluster from all 8 leaves
                    LeafCluster lc_tsdfs, lc_weigh;
                    for (uint8_t leaf_i = 0; leaf_i < 8; leaf_i++) {
                        uint32_t leaf_addr = node[leaf_i];
                        if (leaf_addr == 0) {
                            lc_tsdfs._tsdfs.set_empty(leaf_i);
                            lc_weigh._weigh.set_empty(leaf_i);
                        }
                        else {
                            const auto& leaf = octree.get_leaf(leaf_addr);
                            // weight can be above 255, so we cap it at the uint8_t limit
                            uint8_t weight = std::max<uint8_t>(leaf._weight, std::numeric_limits<uint8_t>::max());
                            lc_tsdfs._tsdfs.set(leaf_i, leaf._signed_distance, sdf_trunc_recip);
                            lc_weigh._weigh.set(leaf_i, weight);
                        }
                    }
                    // add the leaf clusters and remember their addresses
                    nodes_tsdf  [depth][child_i] = node_levels._leaf_clusters.add(lc_tsdfs);
                    nodes_weight[depth][child_i] = node_levels._leaf_clusters.add(lc_weigh);
                }
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
            fmt::println("sub fin  {:.2f}", dur);
        }

        uint32_t root_addr_tsdf;
        uint32_t root_addr_weight;
        std::vector<glm::vec3> positions;
    };
}