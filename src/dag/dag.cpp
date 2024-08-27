#include <chrono>
#include <fmt/base.h>
#include "dag/dag.hpp"
#include "dag/levels.hpp"
#include "dag/morton.hpp"
#include "dag/octree.hpp"

DAG::DAG() {
    // create the main root node (will be empty still)
    // contains 1 header + 8 children
    for (auto i = 0; i < 9; i++) {
        _node_levels[0]._raw_data.push_back(0);
    }
    _node_levels[0]._occupied_count += 9;
    _node_levels[0]._unique_count++;
}
void DAG::insert(std::vector<glm::vec3>& points, glm::vec3 position, glm::quat rotation) {
    auto beg = std::chrono::high_resolution_clock::now();

    // create morton codes to sort points and approx normals
    auto morton_codes = MortonCode::calc(points);
    MortonCode::sort(points, morton_codes);
    auto normals = MortonCode::normals(morton_codes, position);
    // create octree from points and insert into DAG

    Octree octree = octree_build(points);
    insert_octree(octree, points, normals);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
    fmt::println("full dur  {}", dur.count());
}
void DAG::reconstruct() {
}
void DAG::print_stats() {
    for (std::size_t i = 0; i < _node_levels.size(); i++) {
        auto hashset = _node_levels[i]._lookup_set;
        uint64_t hashset_size = hashset.size() / hashset.max_load_factor();
        hashset_size *= sizeof(decltype(hashset)::value_type) + 1;

        fmt::println("level {:2}: {:10} uniques, {:10} dupes, {:.6f} MiB (hashing: {:.6f} MiB)", i, 
            _node_levels[i]._unique_count, 
            _node_levels[i]._dupe_count,
            (double)(_node_levels[i]._occupied_count * sizeof(uint32_t)) / 1024.0 / 1024.0,
            (double)hashset_size / 1024.0 / 1024.0);
    }
    // print same stats for leaf level
    auto hashmap = _leaf_level._lookup_map;
    uint64_t hashset_size = hashmap.size() / hashmap.max_load_factor();
    hashset_size *= sizeof(decltype(hashmap)::value_type) + 1;
    fmt::println("level {:2}: {:10} uniques, {:10} dupes, {:.6f} MiB (hashing: {:.6f} MiB)", _node_levels.size(), 
        _leaf_level._unique_count, 
        _leaf_level._dupe_count,
        (double)(_leaf_level._raw_data.size() * sizeof(LeafLevel::LeafValue)) / 1024.0 / 1024.0,
        (double)hashset_size / 1024.0 / 1024.0);
}
auto DAG::insert_octree(Octree& octree, std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals) -> uint32_t {
    auto beg = std::chrono::steady_clock::now();
    uint32_t root_addr = _node_levels[0]._raw_data.size();

    // trackers that will be updated during traversal
    static constexpr std::size_t max_depth = 63/3 - 1;
    std::array<uint_fast8_t, max_depth> path;
    std::array<std::array<uint32_t, 8>, max_depth> nodes_dag;
    std::array<const Octree::Node*, max_depth> nodes_octree;
    // initialize
    path.fill(0);
    nodes_octree.fill(nullptr);
    for (auto& level: nodes_dag) level.fill(0);
    int_fast32_t depth = 0;
    nodes_octree[0] = octree._root_p;

    // iterate through octree depth-first and insert nodes into DAG bottom-up
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
            auto& level = _node_levels[depth];
            level._raw_data.resize(level._occupied_count + children.size());
            std::memcpy(
                level._raw_data.data() + level._occupied_count,
                children.data(),
                children.size() * sizeof(uint32_t));
            // check if the same node existed previously
            uint32_t temporary = level._occupied_count;
            auto [pIndex, bNew] = level._lookup_set.emplace(temporary);
            if (bNew) {
                level._occupied_count += children.size();
                level._unique_count++;
                if (depth > 0) {
                    nodes_dag[depth - 1][index_in_parent] = temporary;
                }
            }
            else { // TODO: can check for depth>0 earlier
                level._dupe_count++;
                if (depth > 0) {
                    nodes_dag[depth - 1][index_in_parent] = *pIndex;
                }
            }
            depth--;
        }
        // standard node
        else if (depth < (int_fast32_t)max_depth - 1) { // TODO: evaluate (-1 bcs of octree shift)
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

            // check if this leaf cluster already exists
            auto temporary = _leaf_level._raw_data.size();
            auto [pIter, bNew] = _leaf_level._lookup_map.emplace(69696969, temporary);
            if (bNew) {
                _leaf_level._unique_count++;
                _leaf_level._raw_data.push_back(69696969);
                nodes_dag[depth][child_i] = temporary;
            }
            else {
                _leaf_level._dupe_count++;
                // simply update references to the existing cluster
                nodes_dag[depth][child_i] = pIter->second;
            }
        }
    }

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("dag  ins  {:.2f}", dur);
    return root_addr;
}