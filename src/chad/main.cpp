#include <random>
#include <glm/glm.hpp>
#include "chad/tsdf_map.hpp"

[[maybe_unused]]
void inline do_sphere_thing() {
    // generate random point data
    std::vector<glm::vec3> points { 1'000'000 };
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

    // create map with enabled debug outputs
    chad::TSDFMap map{ 0.05f, 0.1f, 3.0f };
    map._debug_outputs = false;

    // insert into CHAD TSDF
    std::vector<glm::vec3> positions {
        { +0, 0, 0 },
        // { +3.5, 0, 0 },
    };
    for (size_t i = 0; i < positions.size(); i++) {
        for (auto& point: points) {
            glm::dvec3 pointd = {
                dis(gen),
                dis(gen),
                dis(gen),
            };
            point = glm::vec3(glm::normalize(pointd) * 5.0);
            point += positions[i];
        }
        map.insert(points, positions[i]);
    }
    map.finalize_active_submap();
    map.print_memory_usage();
    // map.reconstruct("mesh", true);
}
int main() {
    do_sphere_thing();
    return 0;
}
