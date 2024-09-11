#include <array>
#include <vector>
#include <chrono>
#include <cstdint>
#include <fmt/base.h>
#include "dag/dag.hpp"
#include "dag/levels.hpp"
#include "dag/morton.hpp"
#include "dag/octree.hpp"
#include "dag/node.hpp"

auto insert_octree(
    Octree& octree, 
    std::vector<glm::vec3>& points, 
    std::vector<glm::vec3>& normals, 
    std::array<NodeLevel, 63/3-1>& node_levels, 
    LeafLevel& leaf_level) -> uint32_t
{
    auto beg = std::chrono::steady_clock::now();
    fmt::println("NOTE: normalizing avg norm");

    // trackers that will be updated during traversal
    static constexpr std::size_t max_depth = 63/3 - 1;
    std::array<uint_fast8_t, max_depth> path;
    std::array<std::array<uint32_t, 8>, max_depth> nodes_dag;
    std::array<const Octree::Node*, max_depth> nodes_octree;
    uint_fast32_t root_addr = 0xffffffff;
    // initialize
    path.fill(0);
    nodes_octree.fill(nullptr);
    for (auto& level: nodes_dag) level.fill(0);
    nodes_octree[0] = octree._root_p;

    // iterate through octree depth-first and insert nodes into DAG bottom-up
    int_fast32_t depth = 0;
    while (depth >= 0) {
        auto child_i = path[depth]++;

        // when all children were iterated
        if (child_i >= 8) {
            // gather all children for this new node
            std::vector<uint32_t> children(1); // first element is child mask
            for (auto i = 0; i < 8; i++) {
                if (nodes_dag[depth][i] == 0) continue;
                children.push_back(nodes_dag[depth][i]);
                children[0] |= 1 << i; // child mask
            }

            // reset node tracker for handled nodes
            nodes_dag[depth].fill(0);
            // check path in parent depth to know this node's child ID
            uint32_t index_in_parent = path[depth - 1] - 1;

            // resize data if necessary and then copy over
            auto& level = node_levels[depth];
            level._raw_data.resize(level._occupied_count + children.size());
            std::memcpy(
                level._raw_data.data() + level._occupied_count,
                children.data(),
                children.size() * sizeof(uint32_t));
            // check if the same node existed previously
            uint32_t temporary = level._occupied_count;
            auto [index_p, is_new] = level._lookup_set.emplace(temporary);
            if (is_new) {
                level._occupied_count += children.size();
                level._unique_count++;
                if (depth > 0) nodes_dag[depth - 1][index_in_parent] = temporary;
                else root_addr = temporary;
            }
            else {
                level._dupe_count++;
                if (depth > 0) nodes_dag[depth - 1][index_in_parent] = *index_p;
                else root_addr = *index_p;
            }
            depth--;
        }
        // standard node
        else if (depth < (int_fast32_t)max_depth - 1) {
            // retrieve child node
            auto* child_p = nodes_octree[depth]->children[child_i];
            if (child_p == nullptr) continue;
            // walk deeper
            depth++;
            path[depth] = 0;
            nodes_octree[depth] = child_p;
        }
        // leaf cluster
        else {
            // get node containing the points closest to each leaf
            Octree::Node* leaf_cluster = nodes_octree[depth]->children[child_i];
            if (leaf_cluster == nullptr) continue;

            // reconstruct morton code from path
            uint64_t code = 0;
            for (uint64_t k = 0; k < 63/3 - 1; k++) {
                uint64_t part = path[k] - 1;
                code |= part << (60 - k*3);
            }
            // convert into chunk position of leaf cluster
            MortonCode mc(code);
            glm::ivec3 cluster_chunk = mc.decode();

            // create array of signed distances to each leaf
            std::array<float, 8> cluster_sds;
            uint_fast8_t leaf_i = 0;
            for (int z = 0; z <= 1; z++) {
            for (int y = 0; y <= 1; y++) {
            for (int x = 0; x <= 1; x++, leaf_i++) {
                // check leaf validity
                Octree::Node* candidates = leaf_cluster->children[leaf_i];
                if (candidates == nullptr) {
                    // assign invalid leaf value to signify no candidates
                    cluster_sds[leaf_i] = LeafCluster::LEAF_NULL_F;
                    continue;
                }

                // average out the position of candidates and their normals
                glm::vec3 avg_normal{ 0, 0, 0 };
                glm::vec3 avg_pos{ 0, 0, 0 };
                float avg_count = 0;
                for (uint_fast32_t i = 0; i < candidates->leaf_candidates.size(); i++) {
                    if (candidates->leaf_candidates[i] == nullptr) continue;
                    // reverse engineer the index of the current candidate
                    size_t index = candidates->leaf_candidates[i] - points.data();
                    avg_normal += normals[index];
                    avg_pos += *candidates->leaf_candidates[i];
                    avg_count++;
                }

                avg_pos /= avg_count;
                avg_normal /= avg_count;
                avg_normal = glm::normalize(avg_normal); // TODO: test the necessity of this

                // get leaf position
                glm::ivec3 leaf_chunk = cluster_chunk + glm::ivec3(x, y, z);
                glm::vec3 leaf_pos = (glm::vec3)leaf_chunk * (float)LEAF_RESOLUTION;
                // calc signed distance to leaf
                glm::vec3 diff = leaf_pos - avg_pos;
                cluster_sds[leaf_i] = glm::dot(avg_normal, diff);
            }}}
            // compress signed distances into a single 64/32-bit value
            LeafCluster cluster{ cluster_sds };
            // skip clusters that contain only invalid nodes
            if (cluster._value == std::numeric_limits<LeafCluster::ClusterValue>::max()) continue;

            // check if this leaf cluster already exists
            auto temporary_i = leaf_level._raw_data.size();
            auto [it, is_new] = leaf_level._lookup_map.emplace(cluster._value, temporary_i);
            if (is_new) {
                leaf_level._unique_count++;
                leaf_level._raw_data.push_back(cluster._value);
                nodes_dag[depth][child_i] = temporary_i;
            }
            else {
                leaf_level._dupe_count++;
                // simply update references to the existing cluster
                nodes_dag[depth][child_i] = it->second;
            }
        }
    }

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("dag  ctor {:.2f}", dur);
    return root_addr;
}
// merge dag subtree obtained from scan into primary subtree
void merge_primary(uint_fast32_t root_addr, std::array<NodeLevel, 20>& node_levels, LeafLevel& leaf_level) {
    auto beg = std::chrono::steady_clock::now();

    // trackers that will be updated during traversal
    static constexpr std::size_t max_depth = 63/3 - 1;
    std::array<uint_fast8_t, max_depth> path;
    std::array<const Node*, max_depth> nodes_dst;
    std::array<const Node*, max_depth> nodes_src;
    std::array<std::array<uint32_t, 8>, max_depth> nodes_new;
    // initialize
    path.fill(0);
    nodes_dst.fill(nullptr);
    nodes_src.fill(nullptr);
    nodes_dst[0] = Node::from_addr(node_levels[0]._raw_data, 1);
    nodes_src[0] = Node::from_addr(node_levels[0]._raw_data, root_addr);
    for (auto& level: nodes_new) level.fill(0);
    
    // iterate through dag depth-first and merge nodes into primary dag subtree bottom-up
    uint_fast32_t depth = 0;
    while (true) {
        auto child_i = path[depth]++;
        if (child_i == 8) {
            // normal node
            if (depth > 0) {
                // check path in parent depth to know this node's child ID
                uint32_t index_in_parent = path[depth - 1] - 1;

                // check if the node that's about to be created is equal to the current one in dst
                bool equal_nodes = true;
                for (auto i = 0; i < 8; i++) {
                    // compare both child addresses
                    uint32_t addr_new = nodes_new[depth][i];
                    uint32_t addr_dst = 0;
                    // dst node may not always be a valid pointer
                    if (nodes_dst[depth]->contains_child(i)) {
                        addr_dst = nodes_dst[depth]->get_child_addr(i);
                    }
                    if (addr_new != addr_dst) {
                        equal_nodes = false;
                        break;
                    }
                }

                if (equal_nodes) {
                    uint32_t this_node_addr = nodes_dst[depth - 1]->get_child_addr(index_in_parent);
                    nodes_new[depth - 1][index_in_parent] = this_node_addr;
                    // reset node tracker for handled nodes
                    nodes_new[depth].fill(0);
                    depth--;
                    continue;
                }

                // gather all children for this new node
                std::vector<uint32_t> children(1); // first element is child mask
                for (auto i = 0; i < 8; i++) {
                    if (nodes_new[depth][i] == 0) continue;
                    children.push_back(nodes_new[depth][i]);
                    children[0] |= 1 << i; // child mask
                }

                // reset node tracker for handled nodes
                nodes_new[depth].fill(0);

                // resize data if necessary and then copy over
                auto& level = node_levels[depth];
                level._raw_data.resize(level._occupied_count + children.size());
                std::memcpy(
                    level._raw_data.data() + level._occupied_count,
                    children.data(),
                    children.size() * sizeof(uint32_t));
                // check if the same node existed previously
                uint32_t temporary = level._occupied_count;
                auto [index_p, is_new] = level._lookup_set.emplace(temporary);
                if (is_new) {
                    level._occupied_count += children.size();
                    level._unique_count++;
                    nodes_new[depth - 1][index_in_parent] = temporary;
                }
                else {
                    level._dupe_count++;
                    nodes_new[depth - 1][index_in_parent] = *index_p;
                }
                depth--;
            }
            // root node
            else {
                // the root node "dst" will always exist and always has all 8 children
                Node* root_p = Node::from_addr(node_levels[0]._raw_data, 1);
                // simply copy over all children from the new root (prev nodes are preserved)
                for (auto i = 0; i < 8; i++) {
                    root_p->_children[i] = nodes_new[0][i];
                }
                break; // exit main loop
            }
        }
        // normal node
        else if (depth < max_depth - 1) {
            const Node* node_dst_p = nodes_dst[depth];
            const Node* node_src_p = nodes_src[depth];

            // potentially insert new node if present in src tree
            if (node_src_p->contains_child(child_i)) {
                uint32_t child_addr_src = node_src_p->get_child_addr(child_i);

                // check if child is present in dst subtree
                if (node_dst_p->contains_child(child_i)) {
                    uint32_t child_addr_dst = node_dst_p->get_child_addr(child_i);
                    // walk deeper
                    depth++;
                    path[depth] = 0;
                    nodes_dst[depth] = Node::from_addr(node_levels[depth]._raw_data, child_addr_dst);
                    nodes_src[depth] = Node::from_addr(node_levels[depth]._raw_data, child_addr_src);
                }
                // add to dst tree if it does not have a node at this position
                else nodes_new[depth][child_i] = child_addr_src;
            }
            // preserve the already existing node from dst
            else if (node_dst_p->contains_child(child_i)) {
                uint32_t child_addr_dst = node_dst_p->get_child_addr(child_i);
                nodes_new[depth][child_i] = child_addr_dst;
            }
        }
        // leaf cluster node
        else {
            const Node* node_dst_p = nodes_dst[depth];
            const Node* node_src_p = nodes_src[depth];

            // potentially insert new cluster if present in src tree
            if (node_src_p->contains_child(child_i)) {
                uint32_t lc_addr_src = node_src_p->get_child_addr(child_i);
                LeafCluster lc_src = leaf_level._raw_data[lc_addr_src];

                // check if child is present in dst subtree
                if (node_dst_p->contains_child(child_i)) {
                    uint32_t lc_addr_dst = node_dst_p->get_child_addr(child_i);
                    LeafCluster lc_dst = leaf_level._raw_data[lc_addr_dst];

                    // merge leaf clusters
                    LeafCluster merged = LeafCluster::merge(lc_dst, lc_src);
                    // check if merged lc is equal to dst, preserve if so
                    if (merged._value == lc_dst._value) {
                        nodes_new[depth][child_i] = lc_addr_dst;
                    }
                    // if merged lc is not equal, insert as new cluster
                    else {
                        // check if merged lc already exists
                        uint32_t lc_addr_new = leaf_level._raw_data.size();;
                        auto [iter, is_new] = leaf_level._lookup_map.emplace(merged._value, lc_addr_new);
                        if (is_new) {
                            leaf_level._unique_count++;
                            leaf_level._raw_data.push_back(merged._value);
                            nodes_new[depth][child_i] = lc_addr_new;
                        }
                        else {
                            leaf_level._dupe_count++;
                            // simply update references to the existing cluster
                            nodes_new[depth][child_i] = iter->second;
                        }
                    }
                }
                // add to dst tree if it does not have a node at this position
                else {
                    nodes_new[depth][child_i] = lc_addr_src;
                }
            }
            // preserve the already existing node from dst
            else if (node_dst_p->contains_child(child_i)) {
                uint32_t lc_addr_dst = node_dst_p->get_child_addr(child_i);
                nodes_new[depth][child_i] = lc_addr_dst;
            }
        }
    }

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("dag  merg {:.2f}", dur);
}

DAG::DAG(): _node_levels_p(new std::array<NodeLevel, 63/3 - 1>()), _leaf_level_p(new LeafLevel()) {
    // create the main root node (will be empty still)
    for (auto i = 0; i < 9; i++) {
        (*_node_levels_p)[0]._raw_data.push_back(0);
    }
    (*_node_levels_p)[0]._occupied_count += 9;
    (*_node_levels_p)[0]._unique_count++;
    // write root child mask to always contain all 8 children
    (*_node_levels_p)[0]._raw_data[1] = 0xff;
}
// void DAG::insert(std::vector<glm::vec3>& points, glm::vec3 position, glm::quat rotation) {
void DAG::insert(std::array<float, 3>* points_p, std::size_t points_count, std::array<float, 3> position_data, std::array<float, 4> rotation_data) {
    auto beg = std::chrono::high_resolution_clock::now();
    // convert anonymous inputs to named inputs
    glm::vec3 position = { position_data[0], position_data[1], position_data[2] };
    // glm::quat rotation = { rotation_data[0], rotation_data[1], rotation_data[2], rotation_data[3] };
    std::vector<glm::vec3> points { points_count };
    std::memcpy(points.data(), points_p, points_count * sizeof(glm::vec3));

    // create morton codes to sort points and approx normals
    auto morton_codes = MortonCode::calc(points);
    MortonCode::sort(points, morton_codes);
    auto normals = MortonCode::normals(morton_codes, position);
    // create octree from points and insert into DAG
    Octree octree = octree_build(points);
    uint_fast32_t root_addr = insert_octree(octree, points, normals, *_node_levels_p, *_leaf_level_p);
    merge_primary(root_addr, *_node_levels_p, *_leaf_level_p);
    // store the root address of the subtree to keep track
    _subtrees.emplace_back(root_addr);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
    fmt::println("full dur  {}", dur.count());
}
void DAG::print_stats() {
    double total_hashing = 0.0;
    double total_vector = 0.0;
    for (std::size_t i = 0; i < _node_levels_p->size(); i++) {
        auto hashset = (*_node_levels_p)[i]._lookup_set;
        uint64_t hashset_size = hashset.size() / hashset.max_load_factor();
        hashset_size *= sizeof(decltype(hashset)::value_type) + 1;
        double mem_vector = (double)((*_node_levels_p)[i]._raw_data.size() * sizeof(uint32_t)) / 1024.0 / 1024.0;
        double mem_hashing = (double)hashset_size / 1024.0 / 1024.0;
        total_vector += mem_vector;
        total_hashing += mem_hashing;

        fmt::println("level {:2}: {:10} uniques, {:10} dupes, {:.6f} MiB (hashing: {:.6f} MiB)", i, 
            (*_node_levels_p)[i]._unique_count,
            (*_node_levels_p)[i]._dupe_count,
            mem_vector,
            mem_hashing);
    }
    // print same stats for leaf level
    auto hashmap = _leaf_level_p->_lookup_map;
    uint64_t hashset_size = hashmap.size() / hashmap.max_load_factor();
    hashset_size *= sizeof(decltype(hashmap)::value_type) + 1;
    double mem_vector = (double)(_leaf_level_p->_raw_data.size() * sizeof(LeafLevel::ClusterValue)) / 1024.0 / 1024.0;
    double mem_hashing = (double)hashset_size / 1024.0 / 1024.0;
    total_vector += mem_vector;
    total_hashing += mem_hashing;
    fmt::println("level {:2}: {:10} uniques, {:10} dupes, {:.6f} MiB (hashing: {:.6f} MiB)", _node_levels_p->size(), 
        _leaf_level_p->_unique_count, 
        _leaf_level_p->_dupe_count,
        mem_vector,
        mem_hashing);
    fmt::println("total vector memory: {:.6f} MiB", total_vector);
    fmt::println("total hashing memory: {:.6f} MiB", total_hashing);
    fmt::println("total combined memory: {:.6f} MiB", total_vector + total_hashing);
}
auto DAG::get_node_levels() -> std::array<std::vector<uint32_t>*, 63/3 - 1> {
    std::array<std::vector<uint32_t>*, 63/3 - 1> levels;
    for (std::size_t i = 0; i < levels.size(); i++) {
        levels[i] = &(*_node_levels_p)[i]._raw_data;
    }
    return levels;
}
auto DAG::get_leaf_level() -> std::vector<LeafCluster::ClusterValue>& {
    return _leaf_level_p->_raw_data;
}
auto DAG::get_voxel_resolution() -> double {
    return LEAF_RESOLUTION;
}
