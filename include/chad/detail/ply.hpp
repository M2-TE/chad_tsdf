#pragma once
#include "chad/indices.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/dag_storage.hpp"

namespace chad::detail {
    class Ply {
        private:
        struct Vertex {
            void write(std::ofstream& ofs) const {
                ofs.write(reinterpret_cast<const char*>(&_position), sizeof(_position));
                ofs.write(reinterpret_cast<const char*>(&_normal), sizeof(_normal));
                ofs.write(reinterpret_cast<const char*>(&_color), sizeof(_color));
            }
            glm::f32vec3 _position{ 0, 0, 0 };
            glm::f32vec3 _normal{ 0, 0, 0 };
            glm::u8vec3 _color{ 0, 0, 0 };
        };
        struct Face {
            void write(std::ofstream& ofs) const {
                const uint8_t vertcount = 3;
                ofs.write(reinterpret_cast<const char*>(&vertcount), sizeof(vertcount));
                ofs.write(reinterpret_cast<const char*>(&_indices), sizeof(_indices));
            }
            glm::u32vec3 _indices{ 0, 0, 0 };
        };
        struct LeafCopy {
            LeafCopy(float sd): _signed_distance(sd) {}
            // simple raw signed distance from a leaf in f32
            const float _signed_distance;
            // every leaf will have a maximum of 3 vertices placed on +x, +y or +z
            glm::u32vec3 _vertex_indices{ 0, 0, 0 };
        };

        public:
        Ply(const std::string& filename) {
            std::ofstream ofs{ filename, std::ios::binary };

            _ofs << std::string("ply\n");
            _ofs << std::string("format binary_little_endian 1.0\n");
            _ofs << std::format("comment {}\n", COMMENT);
            _ofs << std::string("element vertex                     \n");
            _ofs << std::string("property float32 x\n");
            _ofs << std::string("property float32 y\n");
            _ofs << std::string("property float32 z\n");
            _ofs << std::string("property float32 nx\n");
            _ofs << std::string("property float32 ny\n");
            _ofs << std::string("property float32 nz\n");
            _ofs << std::string("property uint8 red\n");
            _ofs << std::string("property uint8 green\n");
            _ofs << std::string("property uint8 blue\n");
            _ofs << std::string("element face                     \n");
            _ofs << std::string("property list uint8 uint32 vertex_indices\n");
            _ofs << std::string("end_header\n");
        }
        void reconstruct(const DAGStorage& dag, RootIndex tsdf_root, float sdf_res, float sdf_trunc) {
            // TODO: separate lookups for sds and vertices? some leaves will have no sign flips in their neighbourhood and could be omitted
            gtl::parallel_flat_hash_map<MortonCode, LeafCopy> leaves = create_hashmap(dag, tsdf_root, sdf_trunc);

            // now iterate over all hashmap entries to find flipping signs
            for (auto& [key, leaf]: leaves) {
                const glm::ivec3 leaf_voxel = key.decode();
                const glm::vec3 leaf_pos = glm::vec3(leaf_voxel) * sdf_res;

                if (leaf._signed_distance == 0.0f) {
                    // TODO: handle this special case
                    // TODO: add vertex
                    fmt::println("zero");
                    continue;
                }

                // check if the 3 voxels in +x, +y and +z exist and contain a different sign
                const auto leaf_x = leaves.find(MortonCode{ leaf_voxel + glm::ivec3(1, 0, 0) });
                const auto leaf_y = leaves.find(MortonCode{ leaf_voxel + glm::ivec3(0, 1, 0) });
                const auto leaf_z = leaves.find(MortonCode{ leaf_voxel + glm::ivec3(0, 0, 1) });

                if (leaf_x != leaves.cend()) {
                    const float other_sd =  leaf_x->second._signed_distance;
                    // ignore if the other leaf has a signed distance of 0
                    if (other_sd == 0.0f) break;
                    // check if the sign differs
                    if (leaf._signed_distance * other_sd < 0.0f) {
                        // calc vertex position
                        const float other_pos_x = leaf_pos.x + sdf_res;
                        const float final_pos_x = other_pos_x - other_sd * (leaf_pos.x - other_pos_x) / (leaf._signed_distance - other_sd);
                        
                        Vertex v;
                        v._position = leaf_pos;
                        v._position.x = final_pos_x;
                        
                        leaf._vertex_indices.x = 99999999; // TODO
                    }
                }
                if (leaf_y != leaves.cend()) {
                    const float other_sd =  leaf_x->second._signed_distance;
                    // ignore if the other leaf has a signed distance of 0
                    if (other_sd == 0.0f) break;
                    // check if the sign differs
                    if (leaf._signed_distance * other_sd < 0.0f) {
                        // calc vertex position
                        const float other_pos_y = leaf_pos.y + sdf_res;
                        const float interpolation = other_pos_y - other_sd * (leaf_pos.x - other_pos_y) / (leaf._signed_distance - other_sd);
                        fmt::println(" x: {} leaf_x: {} other_x: {}, leaf_sd: {}, other_sd: {}", interpolation, leaf_pos.x, other_pos_y, leaf._signed_distance, other_sd);
                        
                        leaf._vertex_indices.y = 99999999;
                    }
                }
                if (leaf_z != leaves.cend()) {
                    const float other_sd =  leaf_x->second._signed_distance;
                    // ignore if the other leaf has a signed distance of 0
                    if (other_sd == 0.0f) break;
                    // check if the sign differs
                    if (leaf._signed_distance * other_sd < 0.0f) {
                        // calc vertex position
                        const float other_pos_z = leaf_pos.z + sdf_res;
                        const float interpolation = other_pos_z - other_sd * (leaf_pos.x - other_pos_z) / (leaf._signed_distance - other_sd);
                        fmt::println(" x: {} leaf_x: {} other_x: {}, leaf_sd: {}, other_sd: {}", interpolation, leaf_pos.x, other_pos_z, leaf._signed_distance, other_sd);
                        
                        leaf._vertex_indices.z = 99999999;
                    }
                }
            }
        }

        private:
        auto create_hashmap(const DAGStorage& dag, RootIndex tsdf_root, float sdf_trunc) -> gtl::parallel_flat_hash_map<MortonCode, LeafCopy> {
            // track node traversal
            gtl::parallel_flat_hash_map<MortonCode, LeafCopy> leaves;
            std::array<uint32_t, DAGStorage::MAX_DEPTH + 1> path_addrs;
            std::array<uint8_t,  DAGStorage::MAX_DEPTH + 1> path_child_indices;
            path_child_indices.fill(0);
            path_addrs.fill(0);
            // use the root address of the TSDF tree (do not care about weights for reconstruction)
            path_addrs[0] = tsdf_root;
    
            uint32_t depth = 0;
            while (true) {
                uint8_t child_i = path_child_indices[depth]++;
    
                // when all children at this depth were iterated
                if (child_i >= 8) {
                    if (depth > 0) depth--;
                    else break; // exit main loop
                }
    
                // node contains node children
                else if (depth < DAGStorage::MAX_DEPTH - 1) {
                    // try to find the child in current node
                    uint32_t addr = path_addrs[depth];
                    uint32_t child_addr = dag.get_child_addr(depth, addr, child_i);
                    // check if child address is valid
                    if (child_addr > 0) {
                        depth++;
                        path_child_indices[depth] = 0; // reset child index for new depth
                        path_addrs[depth] = child_addr;
                    }
                }
    
                // node contains leaf children
                else {
                    // try to get the leaf cluster, skip if it doesn't exist
                    uint32_t child_addr = dag.get_child_addr(DAGStorage::MAX_DEPTH - 1, path_addrs[depth], child_i);
                    if (child_addr == 0) continue;
    
                    // fetch actual leaf cluster
                    const auto& cluster = dag.get_lc(child_addr);
    
                    // reconstruct morton code from path
                    uint64_t code = 0;
                    for (uint64_t k = 0; k < 63/3 - 1; k++) {
                        uint64_t part = path_child_indices[k] - 1;
                        code |= part << uint64_t(60 - k*3);
                    }
                    MortonCode mc { code };
                    glm::ivec3 cluster_chunk = mc.decode();
    
                    // iterate over all the leaves in the cluster
                    uint32_t leaf_i = 0;
                    for (int32_t z = 0; z <= 1; z++) {
                    for (int32_t y = 0; y <= 1; y++) {
                    for (int32_t x = 0; x <= 1; x++, leaf_i++) {
                        // signed distance within leaf
                        auto [signed_distance, leaf_exists] = cluster._tsdfs.try_get(leaf_i, sdf_trunc);
                        if (!leaf_exists) continue;
    
                        // leaf position
                        glm::ivec3 leaf_chunk = cluster_chunk + glm::ivec3(x, y, z);
                        MortonCode mc_leaf{ leaf_chunk };
                        
                        // add it to the hash map with no vertices yet
                        leaves.emplace(mc_leaf, signed_distance);
                    }}}
                }
            }

            return leaves;
        }

        void dothings() {
            struct Vertex {
                void write(std::ofstream& ofs) const {
                    ofs.write(reinterpret_cast<const char*>(&_position), sizeof(_position));
                    ofs.write(reinterpret_cast<const char*>(&_normal), sizeof(_normal));
                    ofs.write(reinterpret_cast<const char*>(&_color), sizeof(_color));
                }
                glm::f32vec3 _position{ 0, 0, 0 };
                glm::f32vec3 _normal{ 0, 1, 0 };
                glm::u8vec3 _color{ 255, 0, 0 };
            } v;
            struct Face {
                void write(std::ofstream& ofs) const {
                    const uint8_t vertcount = 3;
                    ofs.write(reinterpret_cast<const char*>(&vertcount), sizeof(vertcount));
                    ofs.write(reinterpret_cast<const char*>(&_indices), sizeof(_indices));
                }
                glm::u32vec3 _indices{ 0, 1, 2 };
            } face;

            v._position = { 0, 0, 0 };
            v.write(_ofs);
            v._position = { 1.124, 0, 0 };
            v.write(_ofs);
            v._position = { 0, 1.6713, 0 };
            v.write(_ofs);
            face.write(_ofs);
        }

        private:
        static constexpr std::string_view COMMENT = "Mesh reconstructed by CHAD TSDF";
        std::ofstream _ofs;
    };
}