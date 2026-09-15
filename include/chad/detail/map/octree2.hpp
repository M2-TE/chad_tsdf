#pragma once
#include "chad/detail/dag/storage.hpp"
#include "chad/detail/misc/morton_code.hpp"

namespace chad::detail::map {
    // DEPTH_START (default 17) -> at which depth root nodes will start
    // DEPTH_SPAN  (default  2) -> how many depths a single node spans
    template<std::uint64_t DEPTH_START = 17, std::uint64_t DEPTH_SPAN = 2>
    struct Octree2 {
        static_assert((21 - DEPTH_START) % DEPTH_SPAN == 0);
        using NodeAddr = std::uint32_t;
        struct Leaf {
            float _signed_distance;
            std::uint32_t _weight;
        };
        struct Node {
            constexpr static std::uint64_t DEPTH_CHILDREN = 0b1 << DEPTH_SPAN << DEPTH_SPAN << DEPTH_SPAN;
            union {
                std::array<Leaf, DEPTH_CHILDREN> _leaves;
                std::array<NodeAddr, DEPTH_CHILDREN> _children;
            };
        };

        // default constructor, does nothing
        Octree2() = default;
        // constructs an octree from a hashed DAG tree
        Octree2(const dag::Storage& dag, dag::Addresses root, double sdf_trunc) {
            constexpr std::uint64_t DEPTH_FINAL = dag::Storage::MAX_DEPTH - 2;
            std::array<std::uint8_t,   DEPTH_FINAL + 1> path{}; // child indices along path
            std::array<dag::Addresses, DEPTH_FINAL + 1> addresses{}; // addresses along path
            addresses[0] = root;

            // iterate both trees to build separate octrees
            std::uint32_t depth = 0;
            while (path[0] <= 8) {
                std::uint8_t child_i = path[depth]++;

                if (child_i == 8) {
                    depth--;
                }
                // node contains node children
                else if (depth < DEPTH_FINAL) {
                    // try to find the child in current node
                    dag::ADDR_T child_addr_tsdf = dag.get_node(depth, addresses[depth]._tsdfs, child_i);
                    if (child_addr_tsdf > 0) {
                        // no need to verify since tsdf was valid
                        dag::ADDR_T child_addr_wght = dag.get_node(depth, addresses[depth]._weigh, child_i);
                        // move to next depth
                        depth++;
                        path[depth] = 0; // reset child index for new depth
                        addresses[depth] = {
                            ._tsdfs = child_addr_tsdf,
                            ._weigh = child_addr_wght
                        };
                    }
                }
                // node contains leaf children
                else {
                    // try to find the leaf cluster
                    dag::ADDR_T child_addr_tsdf = dag.get_node(DEPTH_FINAL, addresses[DEPTH_FINAL]._tsdfs, child_i);
                    if (child_addr_tsdf == 0) continue;
                    // no need to verify since tsdf was valid
                    dag::ADDR_T child_addr_wght = dag.get_node(DEPTH_FINAL, addresses[DEPTH_FINAL]._weigh, child_i);

                    // fetch actual leaf cluster
                    const dag::LeafCluster& cluster_tsdf = dag.get_lc(child_addr_tsdf);
                    const dag::LeafCluster& cluster_wght = dag.get_lc(child_addr_wght);

                    // reconstruct morton code from path
                    MortonCode mc{ 0 };
                    for (std::uint64_t k = 0; k < 63/3 - 1; k++) {
                        std::uint64_t part = path[k] - 1;
                        mc._value |= part << static_cast<std::uint64_t>(60 - k*3);
                    }

                    // get the actual leaves
                    std::uint32_t leaf_i = 0;
                    for (std::int8_t z = 0; z <= 1; z++) {
                    for (std::int8_t y = 0; y <= 1; y++) {
                    for (std::int8_t x = 0; x <= 1; x++, leaf_i++) {
                        // signed distance and weight within leaf
                        auto [signed_distance, leaf_exists] = cluster_tsdf._tsdfs.try_get(leaf_i, sdf_trunc);
                        if (!leaf_exists) continue;
                        std::uint8_t weight = cluster_wght._weigh.get(leaf_i);

                        // leaf index will set the 3 LSB
                        std::uint64_t mc_leaf = mc._value | uint64_t(leaf_i);

                        // write the data to new leaf
                        Leaf& leaf = insert(mc_leaf);
                        leaf._signed_distance = signed_distance;
                        leaf._weight = weight;
                    }}}
                }
            }
        }

        // merge another octree into this one via trilinear interpolation
        void merge(const Octree2& octree_b, glm::dmat4x4 transform_b_to_a, double sdf_res) {
            // hardcoded to make development easier for now
            static_assert(DEPTH_START == 17);
            static_assert(DEPTH_SPAN == 2);
            // go over all nodes in b for merging
            for (const auto& [root_mc, root_addr]: octree_b._roots) {
                const Node& root_node = octree_b._nodes[root_addr];
                // go over all children
                for (std::uint64_t child_i = 0; child_i < Node::DEPTH_CHILDREN; child_i++) {
                    NodeAddr child_addr = root_node._children[child_i];
                    if (child_addr == 0) continue;
                    const Node& child_node = octree_b._nodes[child_addr];
                    // go over all leaves
                    for (std::uint64_t leaf_i = 0; leaf_i < Node::DEPTH_CHILDREN; leaf_i++) {
                        const Leaf& leaf = child_node._leaves[leaf_i];
                        if (leaf._weight == 0) continue;

                        // reconstruct morton code from path
                        MortonCode morton_code{ root_mc._value | (child_i << 6) | leaf_i };
                        // voxel position of current leaf from b in coordinate system from b
                        glm::aligned_ivec3 leaf_voxel_b_coord_b = morton_code.decode();

                        // get the other 7 leaves of B to use as trinlinear interpolation input
                        Leaf leaves_b[2][2][2]{};
                        leaves_b[0][0][0] = leaf;
                        for (std::uint8_t z = 0; z <= 1; z++) {
                        for (std::uint8_t y = 0; y <= 1; y++) {
                        for (std::uint8_t x = 0; x <= 1; x++) {
                            // save some time (loop gets unrolled anyways)
                            if (x == 0 && y == 0 && z == 0) continue;
                            // get neighbouring leaf
                            MortonCode mc_other{ leaf_voxel_b_coord_b + glm::aligned_ivec3{ x, y, z }};
                            std::optional<Leaf> leaf_opt = octree_b.find(mc_other);
                            if (leaf_opt.has_value()) leaves_b[x][y][z] = leaf_opt.value();
                        }}}

                        // the goal is to interpolate 8 corner voxels from B (other octree) to position of A (this octree)
                        // so we need to find the leaf A that the 8 leaves from B encompass (initial leaf as the lower left corner [0, 0, 0])
                        // this means that the 8 B leaves will remain grid-aligned, whereas A gets transformed to where it needs to be
                        //
                        //  current:                      o........o
                        // o--------o               o--------A     :
                        // |  A     |  convert to   |     : /|     :
                        // | /      | ------------> |     :/ |     :
                        // |/       |               |     B..|.....o
                        // B--------o               o--------o
                        //

                        // convert to coordinate frame of A
                        glm::aligned_dvec4 leaf_position_b_coord_b{ static_cast<glm::aligned_dvec3>(leaf_voxel_b_coord_b) * sdf_res, 1.0 };
                        glm::aligned_dvec3 leaf_position_b_coord_a{ static_cast<glm::aligned_dvec3>(transform_b_to_a * leaf_position_b_coord_b) };

                        // round up to get the encompassed voxel from A
                        const double sdf_res_reciprocal = 1.0 / sdf_res;
                        // TODO: shouldnt this use flooring instead of ceiling?
                        glm::aligned_dvec3 leaf_voxel_a_coord_a = glm::ceil(leaf_position_b_coord_a * sdf_res_reciprocal);

                        // perform trilinear interpolation (in single-prec float)
                        glm::aligned_dvec3 leaf_position_a_coord_a = leaf_voxel_a_coord_a * sdf_res;
                        glm::aligned_vec3 interpolation_factors = static_cast<glm::aligned_vec3>(leaf_position_a_coord_a - leaf_position_b_coord_a);

                        // get weights
                        float wght_000 = static_cast<float>(leaves_b[0][0][0]._weight);
                        float wght_010 = static_cast<float>(leaves_b[0][1][0]._weight);
                        float wght_001 = static_cast<float>(leaves_b[0][0][1]._weight);
                        float wght_011 = static_cast<float>(leaves_b[0][1][1]._weight);
                        float wght_100 = static_cast<float>(leaves_b[1][0][0]._weight);
                        float wght_110 = static_cast<float>(leaves_b[1][1][0]._weight);
                        float wght_101 = static_cast<float>(leaves_b[1][0][1]._weight);
                        float wght_111 = static_cast<float>(leaves_b[1][1][1]._weight);
                        // get tsdfs (already weighted)
                        float tsdf_000 = leaves_b[0][0][0]._signed_distance * wght_000;
                        float tsdf_010 = leaves_b[0][1][0]._signed_distance * wght_010;
                        float tsdf_001 = leaves_b[0][0][1]._signed_distance * wght_001;
                        float tsdf_011 = leaves_b[0][1][1]._signed_distance * wght_011;
                        float tsdf_100 = leaves_b[1][0][0]._signed_distance * wght_100;
                        float tsdf_110 = leaves_b[1][1][0]._signed_distance * wght_110;
                        float tsdf_101 = leaves_b[1][0][1]._signed_distance * wght_101;
                        float tsdf_111 = leaves_b[1][1][1]._signed_distance * wght_111;

                        // interpolate on x
                        float wght_X00 = std::lerp(wght_000, wght_100, interpolation_factors.x);
                        float wght_X10 = std::lerp(wght_010, wght_110, interpolation_factors.x);
                        float wght_X01 = std::lerp(wght_001, wght_101, interpolation_factors.x);
                        float wght_X11 = std::lerp(wght_011, wght_111, interpolation_factors.x);
                        float tsdf_X00 = std::lerp(tsdf_000, tsdf_100, interpolation_factors.x);
                        float tsdf_X10 = std::lerp(tsdf_010, tsdf_110, interpolation_factors.x);
                        float tsdf_X01 = std::lerp(tsdf_001, tsdf_101, interpolation_factors.x);
                        float tsdf_X11 = std::lerp(tsdf_011, tsdf_111, interpolation_factors.x);
                        // interpolate on y
                        float wght_XY0 = std::lerp(wght_X00, wght_X10, interpolation_factors.y);
                        float wght_XY1 = std::lerp(wght_X01, wght_X11, interpolation_factors.y);
                        float tsdf_XY0 = std::lerp(tsdf_X00, tsdf_X10, interpolation_factors.y);
                        float tsdf_XY1 = std::lerp(tsdf_X01, tsdf_X11, interpolation_factors.y);
                        // interpolate on z
                        float wght_XYZ = std::lerp(wght_XY0, wght_XY1, interpolation_factors.z);
                        float tsdf_XYZ = std::lerp(tsdf_XY0, tsdf_XY1, interpolation_factors.z);

                        // update existing or create new leaf in A
                        Leaf& leaf_a = insert(MortonCode{ static_cast<glm::aligned_ivec3>(leaf_voxel_a_coord_a) });
                        if (leaf_a._weight == 0) {
                            leaf_a._signed_distance = tsdf_XYZ;
                            leaf_a._weight = static_cast<std::uint32_t>(wght_XYZ);
                        }
                        else {
                            leaf_a._signed_distance = leaf_a._signed_distance * static_cast<float>(leaf_a._weight) + tsdf_XYZ; // tsdf_XYZ is already weighted
                            leaf_a._signed_distance /= static_cast<float>(leaf_a._weight) + wght_XYZ;
                            leaf_a._weight = leaf_a._weight + static_cast<std::uint32_t>(wght_XYZ);
                        }
                    }
                }
            }
        }

        void inline clear() {
            _nodes.clear();
            _roots.clear();
        }
        auto inline find(MortonCode morton_code) const -> std::optional<Leaf> {
            // mask out the bits relevant for hashmap lookup
            constexpr std::uint64_t shift_distance = 63 - DEPTH_START * 3;
            constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> shift_distance << shift_distance;

            // obtain node using masked morton code as the key
            auto node_it = _roots.find(morton_code & mask);
            if (node_it == _roots.cend()) return std::nullopt;
            NodeAddr node_addr = node_it->second;

            // walk through each node to reach leaves
            for (std::uint64_t depth = DEPTH_START; depth < 21 - DEPTH_SPAN; depth += DEPTH_SPAN) {
                // walk to next child node
                std::uint64_t child_index = morton_code.child<DEPTH_SPAN>(depth);
                NodeAddr child_addr = _nodes[node_addr]._children[child_index];
                // if one didn't exist yet, return emptyhanded
                if (child_addr == 0) {
                    return std::nullopt;
                }
                node_addr = child_addr;
            }

            // walk to the leaf node
            std::uint64_t leaf_index = morton_code.child<21 - DEPTH_SPAN, DEPTH_SPAN>();
            return _nodes[node_addr]._leaves[leaf_index];
        }
        auto inline insert(MortonCode morton_code) -> Leaf& {
            // mask out the bits relevant for hashmap lookup
            constexpr std::uint64_t shift_distance = 63 - DEPTH_START * 3;
            constexpr std::uint64_t mask = static_cast<std::uint64_t>(-1) >> shift_distance << shift_distance;

            // obtain node using masked morton code as the key
            auto [it, emplaced_b] = _roots.try_emplace(morton_code & mask);
            // if one didn't exist yet, create it
            if (emplaced_b) {
                it->second = _nodes.size();
                _nodes.emplace_back();
            }
            NodeAddr node_addr = it->second;

            // walk through each node to reach leaves
            for (std::uint64_t depth = DEPTH_START; depth < 21 - DEPTH_SPAN; depth += DEPTH_SPAN) {
                // walk to next child node
                std::uint64_t child_index = morton_code.child<DEPTH_SPAN>(depth);
                NodeAddr child_addr = _nodes[node_addr]._children[child_index];
                // if one didn't exist yet, create it
                if (child_addr == 0) {
                    child_addr = _nodes.size();
                    _nodes[node_addr]._children[child_index] = _nodes.size();
                    _nodes.emplace_back();
                }
                node_addr = child_addr;
            }

            // walk to the leaf node
            std::uint64_t leaf_index = morton_code.child<21 - DEPTH_SPAN, DEPTH_SPAN>();
            return _nodes[node_addr]._leaves[leaf_index];
        }
        void inline insert(MortonCode morton_code, float signed_distance) {
            Leaf& leaf = insert(morton_code);

            // weighted merge of signed distance and weight increment
            leaf._signed_distance = leaf._signed_distance * static_cast<float>(leaf._weight) + signed_distance;
            leaf._weight++;
            leaf._signed_distance = leaf._signed_distance / static_cast<float>(leaf._weight);
        }

    public:
        std::vector<Node> _nodes;
        gtl::parallel_flat_hash_map<MortonCode, NodeAddr> _roots; // tree begins at DEPTH_START
        static constexpr std::uint64_t _DEPTH_START = DEPTH_START;
        static constexpr std::uint64_t _DEPTH_SPAN = DEPTH_SPAN;
    };
}
