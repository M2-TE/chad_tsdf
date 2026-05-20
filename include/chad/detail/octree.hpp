#pragma once
#include "chad/detail/dag_storage.hpp"
#include "chad/detail/morton_code.hpp"
#include "chad/detail/virtual_array.hpp"
#include "chad/detail/dag/root_indices.hpp"

namespace chad::detail {
    struct Octree {
        using NodeAddr = uint32_t;
        using Node = std::array<NodeAddr, 8>;
        struct Leaf {
            float _signed_distance = 0.0f;
            uint32_t _weight = 0;
        };

        Octree() {
            _nodes.push_back({ 0, 0, 0, 0, 0, 0, 0, 0 }); // root node
            _leaves.push_back({}); // dummy leaf node
        }

        void clear() {
            _node_lookup.clear();
            _nodes.clear();
            _leaves.clear();

            _nodes.push_back({ 0, 0, 0, 0, 0, 0, 0, 0 }); // root node
            _leaves.push_back({}); // dummy leaf node
        }
        // insert single leaf
        auto inline insert(MortonCode mc) -> Leaf& {
            // see if node at given level has been created already
            static constexpr uint32_t lookup_depth = 18;
            static constexpr uint64_t lookup_shift = (20 - lookup_depth) * 3;
            static constexpr uint64_t lookup_mask = ((0xffffffffffffffff - 1) >> lookup_shift) << lookup_shift;
            auto [node_it, node_emplaced] = _node_lookup.try_emplace(mc._value & lookup_mask, nullptr);

            // start at lookup_depth + 1 when node was found, otherwise start from root
            uint32_t depth = 0;
            Node* node_p = &_nodes[0];
            if (!node_emplaced) depth = lookup_depth + 1;
            if (!node_emplaced) node_p = node_it->second;

            while (depth < 20) {
                uint64_t shift_amount = (20 - depth) * 3; // 3 bits per depth, assuming 21 levels
                uint64_t child_index = (mc._value >> shift_amount) & 0b111;
                uint32_t child_addr = (*node_p)[child_index];
                if (child_addr == 0) {
                    // create new child
                    child_addr = _nodes.size();
                    _nodes.push_back({ 0, 0, 0, 0, 0, 0, 0, 0 });

                    // add the new child to current node
                    (*node_p)[child_index] = child_addr;

                    // if node was missing from lookup table, add it now
                    if (node_emplaced && depth == lookup_depth) {
                        node_it->second = &_nodes[child_addr];
                    }
                }
                // walk to child
                node_p = &_nodes[child_addr];
                depth++;
            }

            uint64_t leaf_index = mc._value & 0b111;
            uint32_t leaf_addr = (*node_p)[leaf_index];
            if (leaf_addr == 0) {
                // create new leaf
                leaf_addr = _leaves.size();
                _leaves.push_back({ 0.0f, 0 });

                // add the new leaf to current node
                (*node_p)[leaf_index] = leaf_addr;
            }

            return _leaves[leaf_addr];
        }
        // insert TSDFs using points and normals (raycasting with DDA in double precision)
        void insert(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const glm::vec3 position, float sdf_res, float sdf_trunc) {
            const double sdf_res_recip = 1.0 / double(sdf_res);
            const glm::aligned_dvec3 position_aligned = position;

            std::vector<MortonCode> traversed_voxels;
            for (size_t i = 0; i < points.size(); i++) {
                const glm::aligned_dvec3 point = points[i];
                const glm::aligned_dvec3 normal = normals[i];

                // calculate ray properties within truncation distance
                const glm::aligned_dvec3 ray_dir = glm::normalize(point - position_aligned);
                const glm::aligned_dvec3 ray_pos = point - ray_dir * double(sdf_trunc);
                const glm::aligned_dvec3 ray_end = point + ray_dir * double(sdf_trunc);
                glm::aligned_ivec3 ray_pos_vox = glm::aligned_ivec3(glm::floor(ray_pos * sdf_res_recip));
                const glm::aligned_ivec3 ray_end_vox = glm::aligned_ivec3(glm::floor(ray_end * sdf_res_recip));

                // the step direction corresponding to ray direction
                const glm::aligned_dvec3 ray_step = glm::sign(ray_dir);
                const glm::aligned_ivec3 ray_step_vox = glm::aligned_ivec3(ray_step);

                // the step distance to reach the next voxel in each dimension
                const glm::aligned_dvec3 ray_delta = glm::abs(double(sdf_res) / ray_dir);

                // the step distance needed to reach the next voxel from current ray_pos
                glm::aligned_dvec3 dim_step = ray_step * (glm::aligned_dvec3(ray_pos_vox) * double(sdf_res) - ray_pos);
                dim_step += (ray_step * 0.5 + 0.5) * double(sdf_res);
                dim_step *= ray_delta * double(sdf_res_recip);

                // can already add the first voxel
                traversed_voxels.emplace_back(ray_pos_vox);

                // 1 bit for each completed dimension
                uint32_t completion_mask = 0b000;
                if (ray_pos_vox.x == ray_end_vox.x) completion_mask |= 0b001;
                if (ray_pos_vox.y == ray_end_vox.y) completion_mask |= 0b010;
                if (ray_pos_vox.z == ray_end_vox.z) completion_mask |= 0b100;

                while (completion_mask != 0b111) {
                    if (dim_step.x < dim_step.y) {
                        if (dim_step.x < dim_step.z) {
                            dim_step.x += ray_delta.x;
                            ray_pos_vox.x += ray_step_vox.x;
                            if (ray_pos_vox.x == ray_end_vox.x) completion_mask |= 0b001;
                        }
                        else {
                            dim_step.z += ray_delta.z;
                            ray_pos_vox.z += ray_step_vox.z;
                            if (ray_pos_vox.z == ray_end_vox.z) completion_mask |= 0b100;
                        }
                    }
                    else {
                        if (dim_step.y < dim_step.z) {
                            dim_step.y += ray_delta.y;
                            ray_pos_vox.y += ray_step_vox.y;
                            if (ray_pos_vox.y == ray_end_vox.y) completion_mask |= 0b010;
                        }
                        else {
                            dim_step.z += ray_delta.z;
                            ray_pos_vox.z += ray_step_vox.z;
                            if (ray_pos_vox.z == ray_end_vox.z) completion_mask |= 0b100;
                        }
                    }
                    traversed_voxels.emplace_back(ray_pos_vox);
                }

                // update the traversed octree leaves
                for (const MortonCode& voxel_mc: traversed_voxels) {
                    auto& leaf = insert(voxel_mc);

                    // compute signed distance
                    glm::aligned_dvec3 voxel_pos = glm::aligned_dvec3(voxel_mc.decode()) + glm::aligned_dvec3(0.5, 0.5, 0.5);
                    glm::aligned_dvec3 point_to_voxel = voxel_pos * double(sdf_res) - point;
                    float signed_distance = float(glm::dot(normal, point_to_voxel));
                    signed_distance = std::clamp(signed_distance, -sdf_trunc, +sdf_trunc);
                    // weighted average with incremented weight
                    leaf._signed_distance = leaf._signed_distance * float(leaf._weight) + signed_distance;
                    leaf._weight++;
                    leaf._signed_distance = leaf._signed_distance / float(leaf._weight);
                }
                traversed_voxels.clear();
            }
        }
        // insert TSDFs from compressed DAG submap
        void insert(const DAGStorage& dag, dag::RootIndices roots, float sdf_trunc) {
            // read-only trackers for submap
            std::array<uint8_t, DAGStorage::MAX_DEPTH> path_child; // child indices along path
            std::array<uint32_t, DAGStorage::MAX_DEPTH> addr_tsdf; // TSDF addresses along path
            std::array<uint32_t, DAGStorage::MAX_DEPTH> addr_wght; // weight addresses along path
            path_child.fill(0);
            addr_tsdf.fill(0);
            addr_wght.fill(0);
            addr_tsdf[0] = roots._tsdfs;
            addr_wght[0] = roots._weights;

            // iterate both trees to build separate octrees
            uint32_t depth = 0;
            while (true) {
                uint8_t child_i = path_child[depth]++;

                // when all children at this depth were iterated
                if (child_i >= 8) {
                    if (depth > 0) depth--;
                    else break; // exit main loop
                }
                // node contains node children
                else if (depth < DAGStorage::MAX_DEPTH - 1) {
                    // try to find the child in current node
                    uint32_t child_addr_tsdf = dag.get_child_addr(depth, addr_tsdf[depth], child_i);

                    // check if child address is valid (only need to check one)
                    if (child_addr_tsdf > 0) {
                        // no need to verify
                        uint32_t child_addr_wght = dag.get_child_addr(depth, addr_wght[depth], child_i);

                        depth++;
                        path_child[depth] = 0; // reset child index for new depth
                        addr_tsdf[depth] = child_addr_tsdf;
                        addr_wght[depth] = child_addr_wght;
                    }
                }
                // node contains leaf children
                else {
                    // try to get the leaf cluster, skip if it doesn't exist
                    uint32_t child_addr_tsdf = dag.get_child_addr(DAGStorage::MAX_DEPTH - 1, addr_tsdf[depth], child_i);
                    if (child_addr_tsdf == 0) continue; // only need to check one
                    uint32_t child_addr_wght = dag.get_child_addr(DAGStorage::MAX_DEPTH - 1, addr_wght[depth], child_i);

                    // fetch actual leaf cluster
                    const LeafCluster& cluster_tsdf = dag.get_lc(child_addr_tsdf);
                    const LeafCluster& cluster_wght = dag.get_lc(child_addr_wght);

                    // reconstruct morton code from path
                    uint64_t code = 0;
                    for (uint64_t k = 0; k < 63/3 - 1; k++) {
                        uint64_t part = path_child[k] - 1;
                        code |= part << uint64_t(60 - k*3);
                    }
                    MortonCode mc{ code };

                    // get the actual leaves
                    uint32_t leaf_i = 0;
                    for (int32_t z = 0; z <= 1; z++) {
                    for (int32_t y = 0; y <= 1; y++) {
                    for (int32_t x = 0; x <= 1; x++, leaf_i++) {
                        // signed distance and weight within leaf
                        auto [signed_distance, leaf_exists] = cluster_tsdf._tsdfs.try_get(leaf_i, sdf_trunc);
                        if (!leaf_exists) continue;
                        uint8_t weight = cluster_wght._weigh.get(leaf_i);

                        // leaf index will set the 3 LSB
                        uint64_t mc_leaf = mc._value | uint64_t(leaf_i);

                        // now just add it
                        insert(mc_leaf) = Leaf{ signed_distance, weight };
                    }}}
                }
            }
        }

        // merge TSDFs from another octree via trilinear interpolation
        void merge(const Octree& octree_b, glm::vec3 delta_b_to_a, float sdf_res) {
            // track node traversal
            std::array<const Octree::Node*, DAGStorage::MAX_DEPTH + 1> path_nodes;
            std::array<uint8_t, DAGStorage::MAX_DEPTH + 1> path_child_indices;
            path_nodes[0] = &octree_b.get_node(Octree::ROOT);
            path_child_indices.fill(0);

            uint32_t depth = 0;
            while (true) {
                uint8_t child_i = path_child_indices[depth]++;

                // when all children at this depth were iterated
                if (child_i >= 8) {
                    if (depth > 0) depth--;
                    else break; // exit main loop
                }

                // node contains node children
                else if (depth < DAGStorage::MAX_DEPTH) {
                    const Octree::Node& node = *path_nodes[depth];
                    uint32_t child_addr = node[child_i];

                    // check if child address is valid
                    if (child_addr > 0) {
                        depth++;
                        path_child_indices[depth] = 0; // reset child index for new depth
                        path_nodes[depth] = &octree_b.get_node(child_addr);
                    }
                }

                // node contains leaf children
                else {
                    const Octree::Node& node = *path_nodes[depth];
                    uint32_t child_addr = node[child_i];
                    if (child_addr == 0) continue;

                    // reconstruct morton code from path
                    uint64_t code = 0;
                    for (uint64_t k = 0; k < DAGStorage::MAX_DEPTH + 1; k++) {
                        uint64_t part = path_child_indices[k] - 1;
                        code |= part << uint64_t(60 - k*3);
                    }
                    MortonCode mc{ code };
                    glm::ivec3 leaf_voxel_b_coord_b = mc.decode();

                    // get the other 7 voxels of B to use as trinlinear interpolation input
                    Octree::Leaf leaves_b[2][2][2];
                    for (int z = 0; z < 2; z++) {
                    for (int y = 0; y < 2; y++) {
                    for (int x = 0; x < 2; x++) {
                        // save some compute
                        if (x == 0 && y == 0 && z == 0) {
                            leaves_b[0][0][0] = octree_b.get_leaf(child_addr);
                            continue;
                        }
                        // get neighbouring leaf voxel position
                        glm::ivec3 leaf_voxel_other = leaf_voxel_b_coord_b + glm::ivec3(x, y, z);
                        MortonCode mc_other{ leaf_voxel_other };

                        // read leaf from octree
                        auto [leaf_p, leaf_exists] = octree_b.try_find(mc_other);
                        if (!leaf_exists) leaves_b[x][y][z] = Octree::Leaf{};
                        else leaves_b[x][y][z] = *leaf_p;
                    }}}

                    // the goal is to interpolate 8 corner voxels from B to position of A
                    // so we need to find the leaf A that leaf B encompasses (B as the lower left corner [0, 0, 0])
                    //  current:                      o........o
                    // o--------o               o--------A     :
                    // |  A     |  convert to   |     : /|     :
                    // | /      | ------------> |     :/ |     :
                    // |/       |               |     B..|.....o
                    // B--------o               o--------o
                    //

                    // convert to coordinate frame of "A"
                    glm::vec3 offset = glm::vec3{1, 1, 1} * sdf_res * 0.01f; // small offset to avoid floating point oddities with glm::ceil
                    glm::vec3 leaf_position_b_coord_b = glm::vec3(leaf_voxel_b_coord_b) * sdf_res;
                    glm::vec3 leaf_position_b_coord_a = leaf_position_b_coord_b + delta_b_to_a - offset;

                    // round up to get the voxel in A that voxel B encompasses
                    const float _sdf_res_recip = 1.0f / sdf_res;
                    glm::vec3 leaf_voxel_a_coord_a = glm::ceil(leaf_position_b_coord_a * _sdf_res_recip);

                    // real position of leaf A to use as the trilinear interpolation target
                    glm::vec3 leaf_position_a_coord_a = leaf_voxel_a_coord_a * sdf_res;

                    // perform trilinear interpolation
                    glm::vec3 interpolation_factors = leaf_position_a_coord_a - leaf_position_b_coord_a;

                    // get weights
                    const float wght_000 = float(leaves_b[0][0][0]._weight);
                    const float wght_010 = float(leaves_b[0][1][0]._weight);
                    const float wght_001 = float(leaves_b[0][0][1]._weight);
                    const float wght_011 = float(leaves_b[0][1][1]._weight);
                    const float wght_100 = float(leaves_b[1][0][0]._weight);
                    const float wght_110 = float(leaves_b[1][1][0]._weight);
                    const float wght_101 = float(leaves_b[1][0][1]._weight);
                    const float wght_111 = float(leaves_b[1][1][1]._weight);
                    // get tsdfs (weighted)
                    const float tsdf_000 = leaves_b[0][0][0]._signed_distance * wght_000;
                    const float tsdf_010 = leaves_b[0][1][0]._signed_distance * wght_010;
                    const float tsdf_001 = leaves_b[0][0][1]._signed_distance * wght_001;
                    const float tsdf_011 = leaves_b[0][1][1]._signed_distance * wght_011;
                    const float tsdf_100 = leaves_b[1][0][0]._signed_distance * wght_100;
                    const float tsdf_110 = leaves_b[1][1][0]._signed_distance * wght_110;
                    const float tsdf_101 = leaves_b[1][0][1]._signed_distance * wght_101;
                    const float tsdf_111 = leaves_b[1][1][1]._signed_distance * wght_111;

                    // interpolate on x
                    const float wght_X00 = std::lerp(wght_000, wght_100, interpolation_factors.x);
                    const float wght_X10 = std::lerp(wght_010, wght_110, interpolation_factors.x);
                    const float wght_X01 = std::lerp(wght_001, wght_101, interpolation_factors.x);
                    const float wght_X11 = std::lerp(wght_011, wght_111, interpolation_factors.x);
                    const float tsdf_X00 = std::lerp(tsdf_000, tsdf_100, interpolation_factors.x);
                    const float tsdf_X10 = std::lerp(tsdf_010, tsdf_110, interpolation_factors.x);
                    const float tsdf_X01 = std::lerp(tsdf_001, tsdf_101, interpolation_factors.x);
                    const float tsdf_X11 = std::lerp(tsdf_011, tsdf_111, interpolation_factors.x);
                    // interpolate on y
                    const float wght_XY0 = std::lerp(wght_X00, wght_X10, interpolation_factors.y);
                    const float wght_XY1 = std::lerp(wght_X01, wght_X11, interpolation_factors.y);
                    const float tsdf_XY0 = std::lerp(tsdf_X00, tsdf_X10, interpolation_factors.y);
                    const float tsdf_XY1 = std::lerp(tsdf_X01, tsdf_X11, interpolation_factors.y);
                    // interpolate on z
                    const float wght_XYZ = std::lerp(wght_XY0, wght_XY1, interpolation_factors.z);
                    const float tsdf_XYZ = std::lerp(tsdf_XY0, tsdf_XY1, interpolation_factors.z);

                    // update existing or create new leaf in A
                    Octree::Leaf& leaf_a = insert(glm::ivec3(leaf_voxel_a_coord_a));
                    leaf_a._signed_distance = leaf_a._signed_distance * float(leaf_a._weight) + tsdf_XYZ; // tsdf_XYZ is already weighted
                    leaf_a._signed_distance /= float(leaf_a._weight) + wght_XYZ;
                    leaf_a._weight = leaf_a._weight + uint32_t(wght_XYZ);
                }
            }
        }

        // try to find specific leaf via morton code
        auto inline try_find(MortonCode mc) const -> std::pair<const Leaf*, bool> {
            // first check if lookup map can find this node
            static constexpr uint32_t lookup_depth = 18;
            static constexpr uint64_t lookup_shift = (20 - lookup_depth) * 3;
            static constexpr uint64_t lookup_mask = ((0xffffffffffffffff - 1) >> lookup_shift) << lookup_shift;
            auto node_it = _node_lookup.find(mc._value & lookup_mask);
            if (node_it == _node_lookup.end()) return { nullptr, false };

            // start at lookup_depth + 1
            uint32_t depth = lookup_depth + 1;
            const Node* node_p = node_it->second;

            // walk a bit further
            while (depth < 20) {
                uint64_t shift_amount = (20 - depth) * 3; // 3 bits per depth, assuming 21 levels
                uint64_t child_index = (mc._value >> shift_amount) & 0b111;
                uint32_t child_addr = (*node_p)[child_index];
                // walk to child
                node_p = &_nodes[child_addr];
                depth++;
            }
            // get leaf node
            uint64_t leaf_index = mc._value & 0b111;
            uint32_t leaf_addr = (*node_p)[leaf_index];
            return { &_leaves[leaf_addr], true };
        }
        auto inline get_node(uint32_t node_addr) const -> const Node& {
            return _nodes[node_addr];
        }
        auto inline get_leaf(uint32_t leaf_addr) -> Leaf& {
            return _leaves[leaf_addr];
        }
        auto inline get_leaf(uint32_t leaf_addr) const -> const Leaf& {
            return _leaves[leaf_addr];
        }

        static constexpr uint32_t ROOT = 0;
        VirtualArray<Node> _nodes;
        VirtualArray<Leaf> _leaves;
        gtl::parallel_flat_hash_map<MortonCode, Node*> _node_lookup; // depth 18
    };
}
