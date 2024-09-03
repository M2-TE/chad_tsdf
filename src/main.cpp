#ifdef CHAD_MAIN
#include <random>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <fmt/base.h>
#include "dag/dag.hpp"

void static read_pcl_file() {
    std::ifstream file;
    file.open("debugpoints.bin", std::ofstream::binary);
    // read number of points present in file
    size_t point_count;
    file.read(reinterpret_cast<char*>(&point_count), sizeof(size_t));
    // read out all points into vector
    std::vector<glm::vec3> points { point_count };
    for (auto& point: points) {
        file.read(reinterpret_cast<char*>(&point), sizeof(glm::vec3));
    }
    fmt::println("Read {} points from file", point_count);
    // insert into DAG and print stats
    DAG dag;
    auto rot = glm::identity<glm::quat>();
    dag.insert(points, glm::vec3(0, 0, 0), rot);
    // dag.print_stats();
}
void static do_sphere_thing() {
    // generate random point data
    std::vector<glm::vec3> points { 100'000 };
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

    // insert into tsdf DAG
    DAG dag;
    for (size_t i = 0; i < 1; i++) {
        for (auto& point: points) {
            glm::dvec3 pointd = {
                dis(gen),
                dis(gen),
                dis(gen)
            };
            pointd = glm::normalize(pointd);
            pointd *= 5.0;
            point = (glm::vec3)pointd;
        }
        dag.insert(points, { 0, 0, 0 }, glm::identity<glm::quat>());
    }
    // dag.print_stats();
}
int main() {
    if (true) read_pcl_file();
    else do_sphere_thing();
}
#endif