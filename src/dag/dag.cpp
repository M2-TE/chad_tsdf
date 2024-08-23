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

void DAG::insert_octree(Octree& octree, std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals) {

}