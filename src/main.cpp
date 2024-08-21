#include <fstream>
#include <glm/glm.hpp>
#include <fmt/base.h>
#include "dag/dag.hpp"

int main() {
    std::ifstream file;
    file.open("debugpoints.bin", std::ofstream::binary);
    // read number of points present in file
    size_t point_count;
    file.read(reinterpret_cast<char*>(&point_count), sizeof(size_t));
    // read out all points into vector
    std::vector<glm::vec3> points(point_count);
    for (auto& point: points) {
        file.read(reinterpret_cast<char*>(&point), sizeof(glm::vec3));
    }
    fmt::println("Read {} points from file", point_count);
    // insert into DAG and print stats
    DAG dag;
    dag.insert(points, { 0, 0, 0 }, glm::identity<glm::quat>());
    dag.print_stats();
    dag.reconstruct();
}