#include <chrono>
#include <fmt/base.h>
#include "dag/dag.hpp"
#include "dag/morton.hpp"
#include "dag/octree.hpp"

DAG::DAG() {
}
void DAG::insert(std::vector<glm::vec3>& points, glm::vec3 position, glm::quat rotation) {
    auto beg = std::chrono::high_resolution_clock::now();

    // create morton codes to sort points and approx normals
    auto morton_codes = morton_code_calc(points);
    morton_code_sort(points, morton_codes);
    auto normals = morton_code_normals(morton_codes, position);
    // create octree from points and insert into DAG
    octree_construct(points);
    
    
    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
    fmt::println("full dur  {}", dur.count());
}
void DAG::reconstruct() {
}
void DAG::print_stats() {
}