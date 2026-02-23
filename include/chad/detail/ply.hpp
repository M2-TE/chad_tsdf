#pragma once
#include "chad/indices.hpp"
#include "chad/detail/mc.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/dag_storage.hpp"

// helpers
namespace {
    struct Vertex {
        void write(std::ofstream& ofs) const {
            ofs.write(reinterpret_cast<const char*>(&_position), sizeof(_position));
            // ofs.write(reinterpret_cast<const char*>(&_normal), sizeof(_normal));
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
        LeafCopy(float sd, uint32_t weight): _signed_distance(sd), _weight(weight) {}
        // simple raw signed distance from a leaf in f32
        const float _signed_distance;
        // keep track of weight simply for coloration
        const uint32_t _weight;
        // every leaf will have a maximum of 3 vertices placed on +x, +y or +z
        glm::u32vec3 _vertex_indices{ 0, 0, 0 };
    };

    static constexpr std::string_view COMMENT = "Mesh reconstructed by CHAD TSDF";
    static constexpr std::string_view PROPERTIES = "\
property float32 x\n\
property float32 y\n\
property float32 z\n\
property uint8 red\n\
property uint8 green\n\
property uint8 blue\n";
// property float32 nx
// property float32 ny
// property float32 nz

    // cheap heatmap color calculation based on cell weights
    auto inline get_gradient_color(uint32_t cell_weight) -> glm::u8vec3 {
        glm::u8vec3 color{ 0, 0, 0 };
        if (cell_weight <= 127) {
            color.b = (127 - cell_weight) * 2;
            color.g = (      cell_weight) * 2;
        }
        else {
            color.g = (127 - (cell_weight - 128)) * 2;
            color.r = (      (cell_weight - 128)) * 2;
        }
        return color;
    }
}

// ply implementation
namespace chad::detail::ply {
    // Step 0: write ply header without vertex/face counts
    void inline write_header(std::ofstream& ofs) {
        ofs << std::string("ply\n");
        ofs << std::string("format binary_little_endian 1.0\n");
        ofs << fmt::format("comment {}\n", COMMENT);
        ofs << std::string("element vertex                     \n");
        ofs << PROPERTIES;
        ofs << std::string("element face                     \n");
        ofs << std::string("property list uint8 uint32 vertex_indices\n");
        ofs << std::string("end_header\n");
    }

    // Step 1 (DAG tree): create a hashmap that is used to perform more efficient neighbour lookups later
    auto inline create_hashmap(const DAGStorage& dag, RootIndices roots, float sdf_trunc) -> gtl::parallel_flat_hash_map<MortonCode, LeafCopy> {
        // read-only trackers for submap
        gtl::parallel_flat_hash_map<MortonCode, LeafCopy> leaves;
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

                    // add it to the hash map with no vertices yet
                    leaves.emplace(mc_leaf, LeafCopy{ signed_distance, uint32_t(weight) });
                }}}
            }
        }

        return leaves;
    }

    // Step 1 (octree): create a hashmap that is used to perform more efficient neighbour lookups later
    auto inline create_hashmap(const Octree& octree) -> gtl::parallel_flat_hash_map<MortonCode, LeafCopy> {
        gtl::parallel_flat_hash_map<MortonCode, LeafCopy> leaves;
        // track node traversal
        std::array<uint8_t, DAGStorage::MAX_DEPTH + 1> path_child;
        std::array<const Octree::Node*, DAGStorage::MAX_DEPTH + 1> path_nodes;
        path_child.fill(0);
        path_nodes.fill(nullptr);
        path_nodes[0] = &octree.get_node(Octree::ROOT);

        uint32_t depth = 0;
        while (true) {
            uint8_t child_i = path_child[depth]++;

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
                    path_child[depth] = 0; // reset child index for new depth
                    path_nodes[depth] = &octree.get_node(child_addr);
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
                    uint64_t part = path_child[k] - 1;
                    code |= part << uint64_t(60 - k*3);
                }
                MortonCode mc{ code };

                // obtain leaf and add it to hashmap
                const Octree::Leaf& leaf = octree.get_leaf(child_addr);
                leaves.emplace(mc, LeafCopy{ leaf._signed_distance, leaf._weight });
            }
        }
        return leaves;
    }

    // Step 2: create vertices at flipping signs
    auto inline create_vertices(std::ofstream& ofs, gtl::parallel_flat_hash_map<MortonCode, LeafCopy>& leaves, float sdf_res) -> uint32_t {
        uint32_t vertex_count = 0;
        for (auto& [mc, leaf]: leaves) {
            const glm::ivec3 leaf_voxel = mc.decode();
            const glm::vec3 leaf_pos = glm::vec3(leaf_voxel) * sdf_res;

            // handle sds of 0 as a special case
            if (leaf._signed_distance == 0.0f) {
                Vertex v;
                v._position = leaf_pos;
                v._color = get_gradient_color(leaf._weight);
                v.write(ofs);

                leaf._vertex_indices.x = vertex_count;
                leaf._vertex_indices.y = vertex_count;
                leaf._vertex_indices.z = vertex_count;
                vertex_count++;
                continue;
            }

            for (uint32_t dimension_i = 0; dimension_i < 3; dimension_i++) {
                // offset by 1 in each dimension
                glm::ivec3 offset{ 0, 0, 0 };
                offset[dimension_i] = 1;

                // check if that leaf exists
                const auto other_it = leaves.find(leaf_voxel + offset);
                if (other_it == leaves.cend()) continue;
                // check for flipping sign
                const float leaf_sd = leaf._signed_distance;
                const float other_sd = other_it->second._signed_distance;
                if (other_sd * leaf_sd >= 0.0f) continue;
                
                const float leaf_weight = float(leaf._weight);
                const float other_weight = float(other_it->second._weight);

                const float leaf_pos_n = leaf_pos[dimension_i];
                const float other_pos_n = leaf_pos[dimension_i] + sdf_res;

                // interpolate position and weight based on signed distances
                const float pos_n = other_pos_n - other_sd * (leaf_pos_n - other_pos_n) / (leaf_sd - other_sd);
                const float weight = other_weight - other_sd * (leaf_weight - other_weight) / (leaf_sd - other_sd);

                // create a single vertex at the interpolated position
                Vertex v;
                v._position = leaf_pos;
                v._position[dimension_i] = pos_n;
                // v._color = get_gradient_color(uint32_t(weight)); // heatmap color
                v._color = glm::u8vec3(0, 255, 0); // DEBUG COLOR
                v.write(ofs);

                leaf._vertex_indices[dimension_i] = vertex_count++;
            }
        }
        return vertex_count;
    }

    // Step 3: create faces with marching cubes lookup table
    // TODO: handle SD of 0.0f properly
    auto inline create_faces(std::ofstream& ofs, gtl::parallel_flat_hash_map<MortonCode, LeafCopy>& leaves) -> uint32_t {
        uint32_t face_count = 0;
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

            // TODO: move this somewhere else
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
                face.write(ofs);
                face_count++;
            }
        }
        return face_count;
    }

    // Step 4: write vertex and face counts into header
    void inline update_header(std::ofstream& ofs, uint32_t vertex_count, uint32_t face_count) {
        ofs.seekp(60  + COMMENT.size());
        ofs << vertex_count;
        ofs.seekp(94 + COMMENT.size() + PROPERTIES.size());
        ofs << face_count;
    }
}

namespace chad::detail::ply {
    // reconstruct ply mesh from octree
    void inline reconstruct(const std::string& filename, const Octree& octree, float sdf_res) {
        std::ofstream ofs{ filename, std::ios::binary };
        if (!ofs.is_open()) fmt::println("Failed to open {} for writing", filename);

        write_header(ofs);
        auto leaves = create_hashmap(octree);
        uint32_t vertex_count = create_vertices(ofs, leaves, sdf_res);
        uint32_t face_count = create_faces(ofs, leaves);
        update_header(ofs, vertex_count, face_count);
        ofs.close();
    }
    // reconstruct ply mesh from hashed DAG tree
    void inline reconstruct(const std::string& filename, const DAGStorage& dag, RootIndices roots, float sdf_res, float sdf_trunc) {
        std::ofstream ofs{ filename, std::ios::binary };
        if (!ofs.is_open()) fmt::println("Failed to open {} for writing", filename); 

        write_header(ofs);
        auto leaves = create_hashmap(dag, roots, sdf_trunc);
        uint32_t vertex_count = create_vertices(ofs, leaves, sdf_res);
        uint32_t face_count = create_faces(ofs, leaves);
        update_header(ofs, vertex_count, face_count);
        ofs.close();
    }
}