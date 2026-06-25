#include <random>
#include <glm/glm.hpp>
#include "chad/tsdf_map.hpp"

[[maybe_unused]]
void inline sample_sphere(std::vector<glm::vec3>& points, glm::vec3 offset, double radius) {
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);
    for (auto& point: points) {
        glm::dvec3 pointd = {
            dis(gen),
            dis(gen),
            dis(gen),
        };
        point = glm::vec3{ glm::normalize(pointd) * radius };
        point += offset;
    }
}

[[maybe_unused]]
void inline sample_cube(std::vector<glm::vec3>& points, glm::vec3 offset) {
    std::random_device rd;
    std::mt19937 gen{ 420 };

    static constexpr float halfsize = 5.0f;
    std::uniform_real_distribution<float> dis_f{ -halfsize, +halfsize };
    std::uniform_int_distribution<uint8_t> dis_i{ 0, 6 };

    for (auto& point: points) {
        uint8_t face_i = dis_i(gen);
        switch (face_i) {
            case 0: point = { dis_f(gen), +halfsize, dis_f(gen) }; break; // top
            case 1: point = { dis_f(gen), -halfsize, dis_f(gen) }; break; // bot
            case 2: point = { +halfsize, dis_f(gen), dis_f(gen) }; break; // rgt
            case 3: point = { -halfsize, dis_f(gen), dis_f(gen) }; break; // lft
            case 4: point = { dis_f(gen), dis_f(gen), +halfsize }; break; // fwd
            case 5: point = { dis_f(gen), dis_f(gen), -halfsize }; break; // bwd
        }
        point += offset;
    }
}

[[maybe_unused]]
void inline do_thingy() {
    std::vector<glm::vec3> points { 10'000'000 };

    // create map with enabled debug outputs
    chad::TSDFMap map{ 0.05f, 0.1f, 3.0f };
    map._debug_outputs = true;

    // insert into CHAD TSDF
    std::vector<glm::vec3> positions {
        { 0.0, 0, 0 },
        { 3.5, 0, 0 },
        { 7.0, 0, 0 },
    };
    for (size_t i = 0; i < positions.size(); i++) {
        sample_sphere(points, positions[i], 20.0);
        // sample_cube(points, positions[i]);
        map.insert(points, positions[i], {});
    }


    // map.finalize_active_submap();

    // // glm::vec3 offset{ 0, 0, 0 };
    // // glm::vec3 offset{ 0.01, 0.01, 0.01 };
    // glm::vec3 offset{ 0.03, 0.03, 0.03 };
    // // glm::vec3 offset{ 0.05, 0.05, 0.05 };
    // // glm::vec3 offset{ 10, 10, 10 };
    // // sample_sphere(points, offset);
    // // sample_cube(points, offset);
    // for (auto& point: points) {
    //     point += offset;
    // }
    // map.dothingy(points, offset);

    // map.print_memory_usage();
    // map.reconstruct("mesh", true);
}
int main() {
    do_thingy();
    return 0;
}
