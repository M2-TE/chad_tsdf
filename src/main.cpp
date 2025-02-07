#ifdef CHAD_MAIN
#include <random>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <fmt/base.h>
#include "chad/chad.hpp"

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
    // insert into CHAD TSDF and print stats
    Chad chad;
    auto rot = glm::identity<glm::quat>();
    chad.insert(points, glm::vec3(0, 0, 0), rot);
    chad.print_stats();
    chad.merge_all_subtrees();
}
void static do_sphere_thing() {
    // generate random point data
    std::vector<glm::vec3> points { 100'000 };
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

    // insert into CHAD TSDF
    Chad chad;
    std::vector<glm::vec3> positions {
        { 10, 10, 10 },
    };
    // insert the same position again n times
    for (size_t i = 0; i < 0; i++) {
        positions.push_back(positions[0]);
    }
    for (size_t i = 0; i < positions.size(); i++) {
        for (auto& point: points) {
            glm::dvec3 pointd = {
                dis(gen),
                dis(gen),
                dis(gen)
            };
            pointd = glm::normalize(pointd);
            pointd *= 5.0;
            point = (glm::vec3)pointd;
            point += positions[i];
        }
        chad.insert(points, positions[i], glm::identity<glm::quat>());
    }
    // dag.merge_all_subtrees();
}
int main() {
    if (false) read_pcl_file();
    else do_sphere_thing();
}
#endif // defined(CHAD_MAIN)