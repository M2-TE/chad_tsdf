#define CHAD_FORCE_GLM
#include "chad/tsdf_map.hpp"

[[maybe_unused]] void static do_sphere_thing() {
    // generate random point data
    std::vector<glm::vec3> points { 1'000'000 };
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

    // insert into CHAD TSDF
    chad::TSDFMap map{ 0.05f, 0.1f, 1.0f };
    std::vector<glm::vec3> positions {
        { 0, 0, 0 },
        { 5, 5, 5 },
    };
    for (size_t i = 0; i < positions.size(); i++) {
        for (auto& point: points) {
            glm::dvec3 pointd = {
                dis(gen),
                dis(gen),
                dis(gen),
            };
            pointd = glm::normalize(pointd);
            pointd *= 5.0;
            point = (glm::vec3)pointd;
            point += positions[i];
        }
        map.insert(points, positions[i]);
    }
    map.reconstruct("mesh.ply");
}
int main() {
    do_sphere_thing();

    // std::vector<glm::vec3> points { 1'000'000 };
    // std::random_device rd;
    // std::mt19937 gen(420);
    // std::uniform_real_distribution<double> dis(-1.0f, 1.0f);
    // for (auto& point: points) {
    //     glm::dvec3 pointd = {
    //         dis(gen),
    //         dis(gen),
    //         dis(gen),
    //     };
    //     pointd = glm::normalize(pointd);
    //     pointd *= 5.0;
    //     point = (glm::vec3)pointd;
    // }

    // auto points_copy = points;
    // glm::vec3 pos{ 99.2465, 99.1231, 99.3564 };
    // // glm::vec3 pos{ 0, 0, 0 };
    // for (auto& point: points_copy) point += pos;

    // // build descriptors
    // ndd::Descriptor desc{ points, glm::vec3{ 0, 0, 0 }};
    // ndd::Descriptor desc_copy{ points_copy, pos };
    // // check if loop closure is found
    // std::vector<ndd::Descriptor> descriptors{ desc, desc_copy };
    // std::vector<ndd::Descriptor::LookupKey> lookup_keys{ desc.get_lookup_key(), desc_copy.get_lookup_key() };
    // ndd::detect_loop_closure(descriptors, lookup_keys, descriptors.size() - 1);

    return 0;
}
