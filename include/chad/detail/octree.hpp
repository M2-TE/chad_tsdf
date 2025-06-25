#pragma once
#include <array>
#include <cstdint>
#include <fmt/base.h>
#include <gtl/phmap.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_aligned.hpp>
#include "chad/detail/morton.hpp"
#include "chad/detail/virtual_array.hpp"

namespace chad::detail {
    struct Octree {
        using NodeAddr = uint32_t;
        using Node = std::array<NodeAddr, 8>;
        struct Leaf { float _signed_distance = 0.0f; uint32_t _weight = 0; };

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
        // insert TSDFs via points and normals
        void insert(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const glm::vec3 position, float sdf_res, float sdf_trunc) {
            auto beg = std::chrono::high_resolution_clock::now();
            const float sdf_res_recip = float(1.0 / double(sdf_res));
            const glm::aligned_vec3 position_aligned = position;

            gtl::flat_hash_set<MortonCode> traversed_voxels;
            for (size_t i = 0; i < points.size(); i++) {
                const glm::aligned_vec3 point = points[i];
                const glm::aligned_vec3 normal = normals[i];

                // get all voxels along ray within truncation distance via variant of DDA line algorithm (-> "A fast voxel traversal algorithm for ray tracing")
                // as Bresehnham's line algorithm misses some voxels
                const glm::aligned_vec3 direction = glm::normalize(point - position_aligned);
                const glm::aligned_vec3 direction_recip = 1.0f / direction;
                const glm::aligned_vec3 start = point - direction * (sdf_trunc);
                const glm::aligned_vec3 final = point + direction * (sdf_trunc);
                const glm::aligned_ivec3 voxel_start = glm::aligned_ivec3(glm::floor(start * sdf_res_recip));
                const glm::aligned_ivec3 voxel_final = glm::aligned_ivec3(glm::floor(final * sdf_res_recip));

                // stepN: direction of increment for each dimension
                const glm::aligned_ivec3 voxel_step_direction = glm::sign(voxel_final - voxel_start);
                // tDeltaN: portion of "direction" needed to traverse full voxel
                const glm::aligned_vec3 voxel_step_delta = glm::abs(sdf_res * direction_recip);
                // tMaxN: portion of "direction" needed to traverse current voxel
                glm::aligned_vec3 voxel_step_max;
                // for x
                if      (voxel_step_direction.x < 0) voxel_step_max.x = sdf_res * std::floor(start.x * sdf_res_recip);
                else if (voxel_step_direction.x > 0) voxel_step_max.x = sdf_res * std::ceil (start.x * sdf_res_recip);
                else /*voxel_step_direction.x == 0*/ voxel_step_max.x = std::numeric_limits<float>::max();
                // for y
                if      (voxel_step_direction.y < 0) voxel_step_max.y = sdf_res * std::floor(start.y * sdf_res_recip);
                else if (voxel_step_direction.y > 0) voxel_step_max.y = sdf_res * std::ceil (start.y * sdf_res_recip);
                else /*voxel_step_direction.x == 0*/ voxel_step_max.y = std::numeric_limits<float>::max();
                // for z
                if      (voxel_step_direction.z < 0) voxel_step_max.z = sdf_res * std::floor(start.z * sdf_res_recip);
                else if (voxel_step_direction.z > 0) voxel_step_max.z = sdf_res * std::ceil (start.z * sdf_res_recip);
                else /*voxel_step_direction.x == 0*/ voxel_step_max.z = std::numeric_limits<float>::max();
                voxel_step_max = voxel_step_max - start; // distance to voxel boundaries
                voxel_step_max = glm::abs(voxel_step_max * direction_recip); // portion of "direction" needed to cross voxel boundaries
                
                // current voxel during traversal
                glm::ivec3 voxel_current = voxel_start;

                // add all 8 corner voxels at start voxel
                for (int8_t x = 0; x < 2; x++) {
                    for (int8_t y = 0; y < 2; y++) {
                        for (int8_t z = 0; z < 2; z++) {
                            MortonCode mc{ voxel_current + glm::ivec3{ x, y, z } };
                            traversed_voxels.emplace(mc);
                        }
                    }
                }

                // traverse ray within truncation distance
                while (true) {
                    if (voxel_step_max.x < voxel_step_max.y) {
                        if (voxel_step_max.x < voxel_step_max.z) {
                            voxel_current.x += voxel_step_direction.x; // step in x direction
                            voxel_step_max.x += voxel_step_delta.x; // update for next voxel boundary
                            if (voxel_current.x == voxel_final.x + voxel_step_direction.x) break;
                        }
                        else {
                            voxel_current.z += voxel_step_direction.z; // step in z direction
                            voxel_step_max.z += voxel_step_delta.z; // update for next voxel boundary
                            if (voxel_current.z == voxel_final.z + voxel_step_direction.z) break;
                        }
                    }
                    else {
                        if (voxel_step_max.y < voxel_step_max.z) {

                            voxel_current.y += voxel_step_direction.y; // step in y direction
                            voxel_step_max.y += voxel_step_delta.y; // update for next voxel boundary
                            if (voxel_current.y == voxel_final.y + voxel_step_direction.y) break;
                        }
                        else {
                            voxel_current.z += voxel_step_direction.z; // step in z direction
                            voxel_step_max.z += voxel_step_delta.z; // update for next voxel boundary
                            if (voxel_current.z == voxel_final.z + voxel_step_direction.z) break;
                        }
                    }

                    // add all 8 corner voxels
                    for (int8_t x = 0; x < 2; x++) {
                        for (int8_t y = 0; y < 2; y++) {
                            for (int8_t z = 0; z < 2; z++) {
                                MortonCode mc{ voxel_current + glm::ivec3{ x, y, z } };
                                traversed_voxels.emplace(mc);
                            }
                        }
                    }
                }
                
                for (const MortonCode& voxel_mc: traversed_voxels) {
                    auto& leaf = insert(voxel_mc);

                    // compute signed distance
                    glm::aligned_vec3 point_to_voxel = glm::aligned_vec3(voxel_mc.decode()) * sdf_res - point;
                    float signed_distance = glm::dot(normal, point_to_voxel);
                    // weighted average with incremented weight
                    leaf._signed_distance = leaf._signed_distance * float(leaf._weight) + signed_distance;
                    leaf._weight++;
                    leaf._signed_distance = leaf._signed_distance / float(leaf._weight);
                }
                traversed_voxels.clear();
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
            fmt::println("oct  upd {:.2f}", dur);
        }

        auto static get_root() -> uint32_t {
            return 0;
        }
        auto inline get_node(uint32_t node_addr) const -> const Node& {
            return _nodes[node_addr];
        }
        auto inline get_leaf(uint32_t leaf_addr) const -> const Leaf& {
            return _leaves[leaf_addr];
        }
        auto inline get_child_addr(uint32_t parent_addr, uint8_t child_i) const -> uint32_t {
            return _nodes[parent_addr][child_i];
        }

        VirtualArray<Node> _nodes;
        VirtualArray<Leaf> _leaves;
        gtl::flat_hash_map<MortonCode, Node*> _node_lookup; // depth 18
    };
}