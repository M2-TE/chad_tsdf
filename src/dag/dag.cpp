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
    // insert_octree(octree, points, normals);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
    fmt::println("full dur  {}", dur.count());
}
void DAG::reconstruct() {
}
void DAG::print_stats() {
    // TODO: add hashset/map memory usage to MiB count
    for (std::size_t i = 0; i < _node_levels.size(); i++) {
        fmt::println("level {:2}: {:10} uniques, {:10} dupes, {:.8f} MiB", i, 
            _node_levels[i]._unique_count, 
            _node_levels[i]._dupe_count,
            (double)(_node_levels[i]._occupied_count * sizeof(uint32_t)) / 1024.0 / 1024.0);
    }
    // print same stats for leaf level
    fmt::println("level {:2}: {:10} uniques, {:10} dupes, {:.8f} MiB", _node_levels.size(), 
        _leaf_level._unique_count, 
        _leaf_level._dupe_count,
        (double)(_leaf_level._raw_data.size() * sizeof(LeafLevel::LeafValue)) / 1024.0 / 1024.0);
}
auto DAG::insert_octree(Octree& octree, std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals) -> uint32_t {
    auto beg = std::chrono::steady_clock::now();
    uint32_t root_addr = _node_levels[0]._raw_data.size();

    // trackers that will be updated during traversal
    std::array<uint_fast8_t, 63/3> path;
    std::array<std::array<uint32_t, 8>, 63/3> nodes_dag;
    std::array<const Octree::Node*, 63/3> nodes_octree;
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
        if (child_i == 8) {
            // TODO
        }
        // standard node
        else if (depth < 63/3 - 1) {
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
            // TODO
        }
    }

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println(" dag  ins {:.2f}", dur);
    return root_addr;
}