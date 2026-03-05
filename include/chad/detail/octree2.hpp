#pragma once
#include "chad/detail/pose.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/submap.hpp"
#include "chad/detail/dag_storage.hpp"
#include "chad/detail/virtual_array.hpp"

namespace chad::detail {
    struct Octree2 {
        using NodeAddr = uint32_t;
        using LeafAddr = uint32_t;
        using Node = std::array<NodeAddr, 8>;
        struct Leaf { float _sd = 0.0f; uint32_t _weight = 0; };

        Octree2() {
            clear();
        }
        void clear() {
            _nodes.clear();
            _leaves.clear();
            _node_map.clear();

            // insert dummy nodes to reserve index 0
            _nodes.push_back({});
            _leaves.push_back({});
        }

        // insert single leaf or return existing one
        auto inline insert(MortonCode mc) -> Leaf& {
            // discretize morton code to match lookup depth to find first node
            auto [node_addr_it, node_emplaced] = _node_map.try_emplace(mc._value & _LOOKUP_MASK, _nodes.size());
            NodeAddr node_addr = node_addr_it->second;
            // create new node when it doesnt exist yet
            if (node_emplaced) _nodes.push_back({ 0, 0, 0, 0, 0, 0, 0, 0 });

            // walk through nodes
            for (uint32_t depth = _NODE_MAP_DEPTH + 1; depth < 20; depth++) {
                uint64_t shift_amount = (20 - depth) * 3; // 3 bits per depth, assuming 21 levels
                uint64_t child_index = (mc._value >> shift_amount) & 0b111;
                NodeAddr child_addr = _nodes[node_addr][child_index];
                // create new child if it doesnt exist yet
                if (child_addr == 0) {
                    child_addr = _nodes.size();
                    _nodes.push_back({ 0, 0, 0, 0, 0, 0, 0, 0 });
                    _nodes[node_addr][child_index] = child_addr;
                }
                // walk to child
                node_addr = child_addr;
            }

            // walk to leaf
            uint64_t leaf_index = mc._value & 0b111;
            LeafAddr leaf_addr = _nodes[node_addr][leaf_index];
            // create new leaf if it doesnt exist yet
            if (leaf_addr == 0) {
                leaf_addr = _leaves.size();
                _leaves.push_back({ 0.0f, 0 });
                _nodes[node_addr][leaf_index] = leaf_addr;
            }
            return _leaves[leaf_addr];
        }
        // insert TSDFs from points and normals (raycasting with DDA in double precision)
        void insert(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const Pose& pose, float sdf_res, float sdf_trunc) {
            const double sdf_res_recip = 1.0 / double(sdf_res);
            const glm::aligned_dvec3 position = pose._position;

            std::vector<MortonCode> traversed_voxels;
            for (size_t i = 0; i < points.size(); i++) {
                const glm::aligned_dvec3 point = points[i];
                const glm::aligned_dvec3 normal = normals[i];

                // calculate ray properties within truncation distance
                const glm::aligned_dvec3 ray_dir = glm::normalize(point - position);
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
                    float sd = float(glm::dot(normal, point_to_voxel));
                    sd = std::clamp(sd, -sdf_trunc, +sdf_trunc);
                    // weighted average with incremented weight
                    leaf._sd = leaf._sd * float(leaf._weight) + sd;
                    leaf._weight++;
                    leaf._sd = leaf._sd / float(leaf._weight);
                }
                traversed_voxels.clear();
            }
        }
        // insert TSDFs from submap (TODO: use try_get for weights instead of tsdfs for checking validity, faster to compute)
        void insert(const DAGStorage& dag, const Submap& submap, float sdf_trunc) {
            // read-only trackers for submap
            std::array<uint8_t, DAGStorage::MAX_DEPTH> path_child; // child indices along path
            std::array<uint32_t, DAGStorage::MAX_DEPTH> addr_tsdf; // TSDF addresses along path
            std::array<uint32_t, DAGStorage::MAX_DEPTH> addr_wght; // weight addresses along path
            path_child.fill(0);
            addr_tsdf.fill(0);
            addr_wght.fill(0);
            addr_tsdf[0] = submap._root_indices._tsdfs;
            addr_wght[0] = submap._root_indices._weights;

            // iterate both trees at once
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
        void merge(const Octree2& octree_b, glm::vec3 delta_b_to_a, float sdf_res) {
            // iterate over all nodes from octree_b
            for (const auto& [base_mc, base_addr]: octree_b._node_map) {
                // walk through nodes
                static_assert(_NODE_MAP_DEPTH == 18); // this just makes my life easier rn
                for (uint64_t child_i = 0; child_i < 8; child_i++) {
                    NodeAddr child_addr = _nodes[base_addr][child_i];

                    // skip nonexitant nodes
                    if (child_addr == 0) continue;

                    // build morton code
                    MortonCode mc = base_mc;
                    mc._value &= child_i << 3;

                    // this node (depth 19) contains 8 leaves (depth 20)
                    const NodeAddr node_addr = child_addr;
                    for (uint64_t leaf_i = 0; leaf_i < 8; leaf_i++) {
                        // TODO -<
                    }
                }
            }
        }
    
        VirtualArray<Node> _nodes;
        VirtualArray<Leaf> _leaves;
        gtl::parallel_flat_hash_map<uint64_t, NodeAddr> _node_map;

        private:
        static constexpr uint32_t _NODE_MAP_DEPTH = 18;
        static constexpr uint64_t _LOOKUP_SHIFT = (20 - _NODE_MAP_DEPTH) * 3; // assuming max depth of 20 (21 levels) and 3 bits per depth
        static constexpr uint64_t _LOOKUP_MASK = ((0xffff'ffff'ffff'ffff - 1) >> _LOOKUP_SHIFT) << _LOOKUP_SHIFT;
    };
}