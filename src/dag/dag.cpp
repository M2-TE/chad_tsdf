#include <chrono>
#include <fmt/base.h>
#include "dag/dag.hpp"
#include "dag/levels.hpp"
#include "dag/morton.hpp"
#include "dag/octree.hpp"

auto insert_octree(Octree& octree, std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals, 
    std::array<NodeLevel, 20>& node_levels, LeafLevel& leaf_level) -> uint32_t
{
    auto beg = std::chrono::steady_clock::now();

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
                glm::vec3 avg_pos { 0, 0, 0 };
                glm::vec3 avg_normal { 0, 0, 0 };
                float avg_count = 0;
                for (uint_fast32_t i = 0; i < candidates->leaf_candidates.size(); i++) {
                    if (candidates->leaf_candidates[i] == nullptr) continue;
                    // reverse engineer the index of the current candidate
                    size_t index = candidates->leaf_candidates[i] - points.data();
                    avg_pos += *candidates->leaf_candidates[i];
                    avg_normal += normals[index];
                    avg_count++;
                }

                avg_pos /= avg_count;
                avg_normal /= avg_normal;
                avg_normal = glm::normalize(avg_normal); // TODO: test the necessity of this

                // get leaf position
                glm::ivec3 leaf_chunk = cluster_chunk + glm::ivec3(x, y, z);
                glm::vec3 leaf_pos = (glm::vec3)leaf_chunk * (float)LEAF_RESOLUTION;
                // calc signed distance to leaf
                glm::vec3 diff = avg_pos - leaf_pos;
                cluster_sds[leaf_i] = glm::dot(diff, avg_normal);
            }}}
            
            // compress signed distances into a single 64/32-bit value
            LeafCluster cluster(cluster_sds);

            // check if this leaf cluster already exists
            auto temporary_i = leaf_level._raw_data.size();
            auto [pIter, bNew] = leaf_level._lookup_map.emplace(cluster._value, temporary_i);
            if (bNew) {
                leaf_level._unique_count++;
                leaf_level._raw_data.push_back(cluster._value);
                nodes_dag[depth][child_i] = temporary_i;
            }
            else {
                leaf_level._dupe_count++;
                // simply update references to the existing cluster
                nodes_dag[depth][child_i] = pIter->second;
            }
        }
    }

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("dag  ctor {:.2f}", dur);
    return root_addr;
}
// pure helper struct for managing DAG nodes, will never be explicitly created or destroyed
struct Node {
    Node() = delete;
    ~Node() = delete;
    Node(const Node&) = delete;
    Node(Node&&) = delete;
    Node& operator=(const Node&) = delete;
    Node& operator=(Node&&) = delete;

    // reinterpret a node address as a valid Node object
    auto static inline from_addr(std::vector<uint32_t>& data, uint_fast32_t addr) -> Node* {
        return reinterpret_cast<Node*>(&data[addr]);
    }
    // check header mask for a specific child
    bool inline contains_child(uint32_t child_i) const {
        uint32_t child_bit = 1 << child_i;
        return _header & child_bit;
    }
    // retrieve child addr from sparse children array
    auto inline get_child_addr(uint32_t child_i) const -> uint32_t {
        // count how many children come before this one
        uint32_t child_bit = 1 << child_i;
        uint32_t masked = _header & (child_bit - 1);
        uint32_t child_count;
#       ifdef CHAD_POPCOUNT_INSTRUCTION
            child_count = CHAD_POPCOUNT_INSTRUCTION(masked);
#       else
            // popcount lookup table: https://stackoverflow.com/a/51388543
            constexpr uint8_t popcount_table[] = {
                0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
            };
            child_count = popcount_table[masked];
#       endif
        return _children[child_count];
    }

    uint32_t _header;
    std::array<uint32_t, 8> _children; // at least 1 valid child, at most 8
};
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

    fmt::println("TODO: check if new node is equal to existing node");
    
    // iterate through dag depth-first and merge nodes into primary dag subtree bottom-up
    uint_fast32_t depth = 0;
    while (true) {
        auto child_i = path[depth]++;
        if (child_i == 8) {
            // normal node
            if (depth > 0) {
                // gather all children for this new node
                std::vector<uint32_t> children(1); // first element is child mask
                for (auto i = 0; i < 8; i++) {
                    if (nodes_new[depth][i] == 0) continue;
                    children.push_back(nodes_new[depth][i]);
                    children[0] |= 1 << i; // child mask
                }

                // reset node tracker for handled nodes
                nodes_new[depth].fill(0);
                // check path in parent depth to know this node's child ID
                uint32_t index_in_parent = path[depth - 1] - 1;

                // TODO: check if new node is equal to the existing one at this tree position

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
                // the root node "dst" will always exist
                // and will already have space allocated for all 8 children
                const Node* root = nodes_dst[0];
                std::array<uint32_t, 9> node_contents;
                for (auto i = 0; i < 8; i++) {
                    uint32_t addr = 0;
                    // first get existing node
                    if (root->contains_child(i)) {
                        addr = root->get_child_addr(i);
                    }
                    // overwrite with new node if present
                    if (nodes_new[depth][i] > 0) {
                        addr = nodes_new[depth][i];
                    }
                    // write child address to node
                    node_contents[i + 1] = addr;
                    // add valid node to child mask
                    if (addr > 0) node_contents[0] |= 1 << i;
                }

                // overwrite old root node
                auto& level = node_levels[0];
                std::memcpy(
                    level._raw_data.data() + 1,
                    node_contents.data(),
                    node_contents.size() * sizeof(uint32_t));
                break; // exit main loop
            }
        }
        // normal node
        else if (depth < max_depth - 1) {
            const Node* node_dst_p = nodes_dst[depth];
            const Node* node_src_p = nodes_src[depth];

            // insert new node if present in src tree but not in dst tree
            if (node_src_p->contains_child(child_i)) {
                uint32_t child_addr_src = node_src_p->get_child_addr(child_i);

                // check if child is present in primary subtree
                if (node_dst_p->contains_child(child_i)) {
                    uint32_t child_addr_dst = node_dst_p->get_child_addr(child_i);
                    // walk deeper
                    depth++;
                    path[depth] = 0;
                    nodes_dst[depth] = Node::from_addr(node_levels[depth]._raw_data, child_addr_dst);
                    nodes_src[depth] = Node::from_addr(node_levels[depth]._raw_data, child_addr_src);
                }
            }
            // preserve the already existing node from dst
            else if (node_dst_p->contains_child(child_i)) {
                uint32_t child_addr_dst = node_dst_p->get_child_addr(child_i);
                nodes_new[depth][child_i] = child_addr_dst;
            }
        }
        // leaf cluster node
        else {
            // TODO
        }
    }

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("dag  merg {:.2f}", dur);
}

DAG::DAG() {
    // create tree levels
    _node_levels_p = new std::array<NodeLevel, 63/3 - 1>();
    _leaf_level_p = new LeafLevel();

    // create the main root node (will be empty still)
    for (auto i = 0; i < 9; i++) {
        (*_node_levels_p)[0]._raw_data.push_back(0);
    }
    (*_node_levels_p)[0]._occupied_count += 9;
    (*_node_levels_p)[0]._unique_count++;
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
    
    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
    fmt::println("full dur  {}", dur.count());
}
void DAG::print_stats() {
    for (std::size_t i = 0; i < _node_levels_p->size(); i++) {
        auto hashset = (*_node_levels_p)[i]._lookup_set;
        uint64_t hashset_size = hashset.size() / hashset.max_load_factor();
        hashset_size *= sizeof(decltype(hashset)::value_type) + 1;

        fmt::println("level {:2}: {:10} uniques, {:10} dupes, {:.6f} MiB (hashing: {:.6f} MiB)", i, 
            (*_node_levels_p)[i]._unique_count,
            (*_node_levels_p)[i]._dupe_count,
            (double)((*_node_levels_p)[i]._occupied_count * sizeof(uint32_t)) / 1024.0 / 1024.0,
            (double)hashset_size / 1024.0 / 1024.0);
    }
    // print same stats for leaf level
    auto hashmap = _leaf_level_p->_lookup_map;
    uint64_t hashset_size = hashmap.size() / hashmap.max_load_factor();
    hashset_size *= sizeof(decltype(hashmap)::value_type) + 1;
    fmt::println("level {:2}: {:10} uniques, {:10} dupes, {:.6f} MiB (hashing: {:.6f} MiB)", _node_levels_p->size(), 
        _leaf_level_p->_unique_count, 
        _leaf_level_p->_dupe_count,
        (double)(_leaf_level_p->_raw_data.size() * sizeof(LeafLevel::ClusterValue)) / 1024.0 / 1024.0,
        (double)hashset_size / 1024.0 / 1024.0);
}
