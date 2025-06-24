#include <random>
#include <fstream>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include "chad/tsdf.hpp"

void static do_sphere_thing() {
    // generate random point data
    std::vector<glm::vec3> points { 100'000 };
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

    // insert into CHAD TSDF
    chad::TSDFMap map;
    std::vector<glm::vec3> positions {
        { 0, 0, 0 },
        // { 2, 2, 2 },
        // { 4, 4, 4 },
        // { 6, 6, 6 },
        // { 8, 8, 8 },
    };
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
        map.insert(points, positions[i]);

        // std::ofstream ofs("points.asc");
        // for (const auto& point: points) {
        //     ofs << point.x << ' ' << point.y << ' ' << point.z << '\n';
        // }
    }
    map.save("mesh.ply");
    map.begin(1);
}
int main() {
    do_sphere_thing();
}