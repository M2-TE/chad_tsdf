#include "dag/dag.hpp"
#include <chrono>
#include <fmt/base.h>
#include "dag/morton.hpp"

DAG::DAG() {
}
void DAG::insert(std::vector<glm::vec3>& points, glm::vec3 position, glm::quat rotation) {
    auto beg = std::chrono::high_resolution_clock::now();

    // create morton codes and sort points with them
    auto morton_codes = morton_code_calc(points);
    morton_code_sort(points, morton_codes);
    // create normals via morton codes
    auto normals = morton_code_normals(morton_codes, position);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg);
    fmt::println("full dur  {}", dur.count());
}
void DAG::reconstruct() {
}
void DAG::print_stats() {
}