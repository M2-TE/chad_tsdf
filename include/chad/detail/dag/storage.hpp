#pragma once
#include "chad/detail/dag/node_level.hpp"
#include "chad/detail/dag/leaf_cluster_level.hpp"
#include "chad/detail/misc/morton_code.hpp"

namespace chad::detail::dag {
    struct Storage {
    private:
        using NodeCache = gtl::flat_hash_map<MortonCode, ADDR_T>;
    public:

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
            for (std::uint32_t depth = 0; depth < MAX_DEPTH - 1; depth++) {
                node_addr = get_node(depth, node_addr, mc.child<1>(depth));
                if (node_addr == 0) return {};
            }
            return get_lc(node_addr);
        }
        // get leaf cluster via MortonCode index
        auto inline get_lc(const NodeCache& cache, MortonCode mc) const -> LeafCluster {
            auto node_addr_it = cache.find(mc.mask<_cache_depth>());
            if (node_addr_it == cache.cend()) return {};

            ADDR_T node_addr = node_addr_it->second;
            for (std::uint32_t depth = _cache_depth; depth < MAX_DEPTH - 1; depth++) {
                node_addr = get_node(depth, node_addr, mc.child<1>(depth));
                if (node_addr == 0) return {};
            }
            return get_lc(node_addr);
        }

        // return single tsdf leaf from morton code index
        auto inline get_tsdf_leaf(ADDR_T root_addr, MortonCode mc, float sdf_trunc) const -> std::pair<float, bool> {
            LeafCluster lc = get_lc(root_addr, mc);
            return lc._tsdfs.try_get(mc.child<MAX_DEPTH - 1, 1>(), sdf_trunc);
        }
        // return single tsdf leaf via morton code
        auto inline get_tsdf_leaf(const NodeCache& cache, MortonCode mc, float sdf_trunc) const -> std::pair<float, bool> {
            LeafCluster lc = get_lc(cache, mc);
            return lc._tsdfs.try_get(mc.child<MAX_DEPTH - 1, 1>(), sdf_trunc);
        }

        // build cache of nodes for a given tree
        auto build_cache(ADDR_T root) /*const*/ -> NodeCache { // TODO: MAKE CONST AGAIN
            std::array<std::uint8_t,  _cache_depth> path{}; // child indices along path
            std::array<ADDR_T, _cache_depth> addresses{}; // addresses along path
            addresses[0] = root;
            NodeCache cache;

            // iterate both trees to build separate octrees
            std::uint32_t depth = 0;
            while (path[0] <= 8) {
                std::uint8_t child_i = path[depth]++;

                if (child_i == 8) {
                    depth--;
                }
                else if (depth < _cache_depth - 1) {
                    // try to find the child in current node
                    ADDR_T child_addr = get_node(depth, addresses[depth], child_i);
                    if (child_addr == 0) continue;

                    depth++;
                    path[depth] = 0; // reset child index for new depth
                    addresses[depth] = child_addr;
                }
                else {
                    ADDR_T child_addr = get_node(depth, addresses[depth], child_i);
                    if (child_addr == 0) continue;

                    // reconstruct morton code from path
                    MortonCode mc{ 0 };
                    for (std::uint64_t k = 0; k < _cache_depth; k++) {
                        std::uint64_t part = path[k] - 1;
                        mc._value |= part << static_cast<std::uint64_t>(60 - k*3);
                    }

                    // add the node to our temporary cache
                    cache[mc] = child_addr;
                }
            }
            return cache;
        }
        // the idea here is to check for signed distance sign flips and create points there (look only in positive axis direction)
        void inline sample_points_from_tsdf_leaves(const NodeCache& cache, std::vector<glm::aligned_vec3>& points, MortonCode mc, LeafCluster lc, float sdf_res, float sdf_trunc) const {
            // get position of the entire leaf cluster, would be inefficient to do this for every leaf individually
            glm::aligned_ivec3 lc_pos_vox = mc.decode();
            glm::aligned_vec3  lc_pos = static_cast<glm::aligned_vec3>(lc_pos_vox) * sdf_res;

            // storage for 2x2x2 main leaves and the rest for adjacent ones from other LCs
            std::pair<float, bool> leaves[3][3][3]{};

            // get the main leaves
            std::uint8_t leaf_i = 0;
            for (std::uint8_t z = 0; z < 2; z++) {
            for (std::uint8_t y = 0; y < 2; y++) {
            for (std::uint8_t x = 0; x < 2; x++, leaf_i++) {
                // get existance and, if it exists, signed distance of leaf
                leaves[x][y][z] = lc._tsdfs.try_get(leaf_i, sdf_trunc);
            }}}

            // get adjacent leaves
            for (std::uint8_t axis_i = 0; axis_i < 3; axis_i++) {
                // get the leaf cluster in that axis direction
                glm::aligned_ivec3 lc_pos_vox_other = lc_pos_vox;
                lc_pos_vox_other[axis_i] += 2; // +2 since we are still at leaf cluster level
                LeafCluster lc_other = get_lc(cache, MortonCode{ lc_pos_vox_other });
                if (lc_other._tsdfs.empty()) continue;

                // write the correct values to "leaves", dont need all 8
                std::uint8_t leaf_i = 0;
                for (std::uint8_t z = 0; z < 2; z++) {
                for (std::uint8_t y = 0; y < 2; y++) {
                for (std::uint8_t x = 0; x < 2; x++, leaf_i++) {
                    glm::vec<3, uint8_t> index = { x, y, z };
                    if (index[axis_i] == 1) continue; // only need adjacent leaves

                    // write the adjacent leaves for later
                    index[axis_i] += 2;
                    leaves[index.x][index.y][index.z] = lc_other._tsdfs.try_get(leaf_i, sdf_trunc);
                }}}
            }

            // place points between leaves with flipping signs
            for (std::uint8_t z = 0; z < 2; z++) {
            for (std::uint8_t y = 0; y < 2; y++) {
            for (std::uint8_t x = 0; x < 2; x++) {
                auto [leaf_sd, leaf_exists] = leaves[x][y][z];
                if (!leaf_exists) continue;

                // get real position of current leaf
                glm::aligned_vec3 leaf_pos{ lc_pos };
                leaf_pos += glm::aligned_vec3{ x, y, z } * sdf_res;

                // special handling for sd of 0
                if (leaf_sd == 0.0f) {
                    leaf_pos += sdf_res * 0.5f; // account for sd calc offset
                    points.push_back(leaf_pos);
                    continue;
                }

                // potentially create points in positive axis directions
                for (std::uint8_t axis = 0; axis < 3; axis++) {
                    std::array<uint8_t, 3> index = { x, y, z };
                    index[axis] += 1;

                    // grab the leaf thats in this axis direction
                    auto [other_sd, other_exists] = leaves[index[0]][index[1]][index[2]];
                    if (!other_exists) continue;

                    // check for flipping sign
                    if (leaf_sd * other_sd >= 0.0f) continue;

                    // position of other leaf will simply need voxel resolution added to the correct axis
                    float leaf_pos_axis = leaf_pos[axis];
                    float other_pos_axis = leaf_pos[axis] + sdf_res;

                    // interpolate position on axis based on signed distances
                    float pos_axis = other_pos_axis - other_sd * (leaf_pos_axis - other_pos_axis) / (leaf_sd - other_sd);

                    // now just put it all together
                    glm::aligned_vec3 point_pos = leaf_pos;
                    point_pos[axis] = pos_axis;

                    point_pos += sdf_res * 0.5f; // account for sd calc offset
                    points.push_back(point_pos);
                }
            }}}
        }
        // return points that lie inbetween flipping signs
        auto sample_points_from_tsdf(ADDR_T tsdf_root, float sdf_res, float sdf_trunc) -> std::vector<glm::aligned_vec3> const {
            constexpr std::size_t FINAL_DEPTH = MAX_DEPTH - 2;
            std::array<std::uint8_t, FINAL_DEPTH - _cache_depth + 1> path{}; // child indices along path
            std::array<ADDR_T,       FINAL_DEPTH - _cache_depth + 1> addresses{}; // addresses along path

            // writes nodes at a certain depth to cache for faster access
            NodeCache cache = build_cache(tsdf_root);

            // write all points into a simple vector
            std::vector<glm::aligned_vec3> points;

            // go over every node currently in the cache
            for (const auto& [cache_mc, cache_node_addr]: cache) {
                path.fill(0);
                addresses.fill(0);
                addresses[0] = cache_node_addr;

                // same iteration logic as with full tree iteration
                std::uint32_t depth = _cache_depth;
                while (path[0] <= 8) {
                    std::uint8_t child_i = path[depth - _cache_depth]++;

                    if (child_i == 8) {
                        depth--;
                    }
                    else if (depth < FINAL_DEPTH) {
                        // try to find the child in current node
                        ADDR_T child_addr = get_node(depth, addresses[depth - _cache_depth], child_i);
                        if (child_addr == 0) continue;

                        depth++;
                        path[depth - _cache_depth] = 0; // reset child index for new depth
                        addresses[depth - _cache_depth] = child_addr;
                    }
                    else {
                        ADDR_T child_addr = get_node(FINAL_DEPTH, addresses[FINAL_DEPTH - _cache_depth], child_i);
                        if (child_addr == 0) continue;

                        // fetch actual leaf cluster
                        const LeafCluster& lc = get_lc(child_addr);

                        // reconstruct morton code from path
                        MortonCode mc{ cache_mc._value };
                        for (std::uint64_t k = _cache_depth; k <= FINAL_DEPTH; k++) {
                            std::uint64_t part = path[k - _cache_depth] - 1;
                            mc._value |= part << static_cast<std::uint64_t>(60 - k*3);
                        }

                        // handle the leaves in a separate function
                        sample_points_from_tsdf_leaves(cache, points, mc, lc, sdf_res, sdf_trunc);
                    }
                }
            }
            return points;
        }

        static constexpr std::uint64_t MAX_DEPTH = 21; // TODO: name should be adjusted, this is the max NUMBER of depths
        static constexpr std::size_t _cache_depth = 17; // which depth to perform the caching of points at (see build_cache())
        // 20 levels of standard nodes
        std::array<NodeLevel, MAX_DEPTH - 1> _node_levels;
        // 1 level of leaf clusters
        LeafClusterLevel _leaf_cluster_level;
        // for synchronization during async operations
        std::mutex _mutex;
    };
};
