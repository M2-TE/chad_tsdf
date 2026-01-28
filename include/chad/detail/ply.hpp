#pragma once
#include "chad/indices.hpp"
#include "chad/detail/mc.hpp"
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
        Ply(const std::string& filename): _vertex_count(0), _face_count(0) {
            _ofs.open(filename, std::ios::binary);
            if (!_ofs.is_open()) fmt::println("Failed to open {} for writing", filename);

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
            gtl::parallel_flat_hash_map<MortonCode, LeafCopy> leaves = create_hashmap(dag, tsdf_root, sdf_trunc);

            // create vertices at flipping signs
            for (auto& [mc, leaf]: leaves) {
                const glm::ivec3 leaf_voxel = mc.decode();
                const glm::vec3 leaf_pos = glm::vec3(leaf_voxel) * sdf_res;

                // handle sds of 0 as a special case
                if (leaf._signed_distance == 0.0f) {
                    Vertex v;
                    v._position = leaf_pos;
                    v.write(_ofs);

                    leaf._vertex_indices.x = _vertex_count;
                    leaf._vertex_indices.y = _vertex_count;
                    leaf._vertex_indices.z = _vertex_count;
                    _vertex_count++;
                    continue;
                }

                // check if the 3 voxels in +x, +y and +z exist and contain a different sign
                const auto leaf_x = leaves.find(leaf_voxel + glm::ivec3(1, 0, 0));
                const auto leaf_y = leaves.find(leaf_voxel + glm::ivec3(0, 1, 0));
                const auto leaf_z = leaves.find(leaf_voxel + glm::ivec3(0, 0, 1));

                if (leaf_x != leaves.cend()) {
                    const float other_sd =  leaf_x->second._signed_distance;
                    // check if the sign differs, also ignores other_sd of 0
                    if (leaf._signed_distance * other_sd < 0.0f) {
                        // calc vertex position
                        const float other_pos_x = leaf_pos.x + sdf_res;
                        const float final_pos_x = other_pos_x - other_sd * (leaf_pos.x - other_pos_x) / (leaf._signed_distance - other_sd);
                        
                        Vertex v;
                        v._position = leaf_pos;
                        v._position.x = final_pos_x;
                        v.write(_ofs);
                        
                        leaf._vertex_indices.x = _vertex_count++;
                    }
                }
                if (leaf_y != leaves.cend()) {
                    const float other_sd =  leaf_y->second._signed_distance;
                    // check if the sign differs, also ignores other_sd of 0
                    if (leaf._signed_distance * other_sd < 0.0f) {
                        // calc vertex position
                        const float other_pos_y = leaf_pos.y + sdf_res;
                        const float final_pos_y = other_pos_y - other_sd * (leaf_pos.y - other_pos_y) / (leaf._signed_distance - other_sd);

                        Vertex v;
                        v._position = leaf_pos;
                        v._position.y = final_pos_y;
                        v.write(_ofs);
                        
                        leaf._vertex_indices.y = _vertex_count++;
                    }
                }
                if (leaf_z != leaves.cend()) {
                    const float other_sd =  leaf_z->second._signed_distance;
                    // check if the sign differs, also ignores other_sd of 0
                    if (leaf._signed_distance * other_sd < 0.0f) {
                        // calc vertex position
                        const float other_pos_z = leaf_pos.z + sdf_res;
                        const float final_pos_z = other_pos_z - other_sd * (leaf_pos.z - other_pos_z) / (leaf._signed_distance - other_sd);

                        Vertex v;
                        v._position = leaf_pos;
                        v._position.z = final_pos_z;
                        v.write(_ofs);

                        leaf._vertex_indices.z = _vertex_count++;
                    }
                }
            }

            // create indices as per marching cubes LUT
            // TODO: handle SD of 0.0f
            // TODO: handle missing corners (should still be able to create faces)
            for (const auto& [mc000, leaf000]: leaves) {
                const glm::ivec3 pos000 = mc000.decode();

                // vertex and edge indexing:
                // TODO: FIX
                //          v7----e6-----v6
                //        / |           / |
                //     e11 e7        e10 e5
                //   v3------e2----v2     |
                //    |     v4---e4-+----v5
                //   e3  e8        e1  e9
                //    | /           | /
                //   v0-----e0-----v1
                //
                // with v0 at (0, 0, 0) and v6 at (1, 1, 1)
                
                // the current leaf will be the [0, 0, 0] of this voxel
                // fetch the other 6 leaves to get information on all 12 voxel edges
                // for now just ignore cubes with missing corners
                const auto it001 = leaves.find(pos000 + glm::ivec3(0, 0, 1));
                if (it001 == leaves.cend()) continue;
                const auto it100 = leaves.find(pos000 + glm::ivec3(1, 0, 0));
                if (it100 == leaves.cend()) continue;
                const auto it101 = leaves.find(pos000 + glm::ivec3(1, 0, 1));
                if (it101 == leaves.cend()) continue;
                const auto it010 = leaves.find(pos000 + glm::ivec3(0, 1, 0));
                if (it010 == leaves.cend()) continue;
                const auto it011 = leaves.find(pos000 + glm::ivec3(0, 1, 1));
                if (it011 == leaves.cend()) continue;
                const auto it110 = leaves.find(pos000 + glm::ivec3(1, 1, 0));
                if (it110 == leaves.cend()) continue;
                const auto it111 = leaves.find(pos000 + glm::ivec3(1, 1, 1));
                if (it111 == leaves.cend()) continue;

                // calling it corners to not confuse it with the actual mesh vertices
                const std::array<LeafCopy, 8> corners {
                    leaf000,       // 0
                    it100->second, // 1
                    it010->second, // 2
                    it110->second, // 3
                    it001->second, // 4
                    it101->second, // 5
                    it011->second, // 6
                    it111->second, // 7
                };

                // take vertex indices for every edge
                const std::array<uint32_t, 12> edges {
                    corners[0]._vertex_indices.x,
                    corners[1]._vertex_indices.y,
                    corners[2]._vertex_indices.x,
                    corners[0]._vertex_indices.y,

                    corners[4]._vertex_indices.x,
                    corners[5]._vertex_indices.y,
                    corners[6]._vertex_indices.x,
                    corners[4]._vertex_indices.y,

                    corners[0]._vertex_indices.z,
                    corners[1]._vertex_indices.z,
                    corners[3]._vertex_indices.z,
                    corners[2]._vertex_indices.z,
                };

                // DEBUG: TEMPORARILY DISABLE CUBES WITH SD OF 0.0f
                // bool breaking = false;
                // for (uint32_t i = 0; i < 8; i++) {
                //     if (corners[i]._signed_distance == 0.0f)  breaking = true;
                // }
                // if (breaking) continue;

                // create the lookup index for the marching cubes table
                uint32_t marching_cubes_index = 0;
                for (uint32_t i = 0; i < 8; i++) {
                    if (corners[i]._signed_distance > 0.0f) {
                        marching_cubes_index |= 1 << i;
                    }
                }
                const std::array<uint32_t, 13>& table_entry = MC_TABLE[marching_cubes_index];
                
                // find out how many vertices are needed
                uint32_t table_entry_length = 0;
                for (uint32_t i = 0; i < table_entry.size(); i++) {
                    if (table_entry[i] == chad::detail::NO) {
                        table_entry_length = i;
                        break;
                    }
                }

                static constexpr std::array<std::pair<uint32_t, uint32_t>, 12> edge_indices = {
                    std::pair<uint32_t, uint32_t>{ 0, 1 },
                    std::pair<uint32_t, uint32_t>{ 1, 3 },
                    std::pair<uint32_t, uint32_t>{ 3, 2 },
                    std::pair<uint32_t, uint32_t>{ 2, 0 },
                    std::pair<uint32_t, uint32_t>{ 4, 5 },
                    std::pair<uint32_t, uint32_t>{ 5, 7 },
                    std::pair<uint32_t, uint32_t>{ 7, 6 },
                    std::pair<uint32_t, uint32_t>{ 6, 4 },
                    std::pair<uint32_t, uint32_t>{ 0, 4 },
                    std::pair<uint32_t, uint32_t>{ 1, 5 },
                    std::pair<uint32_t, uint32_t>{ 3, 7 },
                    std::pair<uint32_t, uint32_t>{ 2, 6 },
                };
                
                // create the faces
                for (uint32_t i = 0; i < table_entry_length; i += 3) {
                    Face face;

                    // fetch the correct vertices
                    for (uint32_t vertex_i = 0; vertex_i < 3; vertex_i++) {
                        uint32_t edge_i = table_entry[i + vertex_i];

                        // check if any corner connected to chosen edge has a SD of 0.0f
                        auto& corner_0 = corners[edge_indices[edge_i].first];
                        auto& corner_1 = corners[edge_indices[edge_i].second];

                        if (corner_0._signed_distance == 0.0f) {
                            face._indices[vertex_i] = corner_0._vertex_indices[0];
                        }
                        else if (corner_1._signed_distance == 0.0f) {
                            face._indices[vertex_i] = corner_1._vertex_indices[0];
                        }
                        else {
                            face._indices[vertex_i] = edges[table_entry[i + vertex_i]];
                        }
                    }

                    // filter out invisible faces
                    if (face._indices[0] == face._indices[1] || 
                        face._indices[0] == face._indices[2] || 
                        face._indices[1] == face._indices[2]) {
                        continue;
                    }

                    // write to ply file
                    face.write(_ofs);
                    _face_count++;
                }
            }
        }
        void finalize() { // just don't look inside
            // write vertex and face counts into header
            _ofs.seekp(60  + COMMENT.size());
            _ofs << _vertex_count;
            _ofs.seekp(271 + COMMENT.size());
            _ofs << _face_count;
            _ofs.close();
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

        private:
        static constexpr std::string_view COMMENT = "Mesh reconstructed by CHAD TSDF";
        std::ofstream _ofs;
        uint32_t _vertex_count; // offset by +1 as index 0 is reserved
        uint32_t _face_count;
    };
}