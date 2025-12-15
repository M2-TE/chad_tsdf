#include "chad/tsdf.hpp"

// #include <Eigen/Eigen> // DEBUG
#include "chad/detail/ndd.hpp"


[[maybe_unused]] void static do_sphere_thing() {
    // generate random point data
    std::vector<glm::vec3> points { 1'000'000 };
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

    // insert into CHAD TSDF
    chad::TSDFMap map{ 0.05f, 0.1f };
    std::vector<glm::vec3> positions {
        { 0, 0, 0 },
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
    map.reconstruct("mesh.ply");
}
[[maybe_unused]] void static double_sphere_thing() {
    chad::TSDFMap map { 0.05f, 0.1f };

    // generate random point data
    std::vector<glm::vec3> points { 1'000'000 };
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);

    // insert first sphere into CHAD TSDF
    glm::vec3 position { 0, 0, 0 };
    for (auto& point: points) {
        glm::dvec3 pointd = {
            dis(gen),
            dis(gen),
            dis(gen),
        };
        pointd = glm::normalize(pointd);
        pointd *= 5.0;
        point = (glm::vec3)pointd;
        point += position;
    }
    map.insert(points, position);
    map.finalize_active_submap();

    // insert second sphere into CHAD TSDF
    glm::vec3 error = {0, 0, 0};
    position = glm::vec3{5, 5, 5};
    for (auto& point: points) {
        glm::dvec3 pointd = {
            dis(gen),
            dis(gen),
            dis(gen),
        };
        pointd = glm::normalize(pointd);
        pointd *= 5.0;
        point = (glm::vec3)pointd;
        point += position + error;
    }
    map.insert(points, position + error);
    map.finalize_active_submap();
    map.get_submap(1)._pose_err = {
        { error.x, error.y, error.z },
        {}
    };

    // merge all
    map.reconstruct("single.ply");
    chad::Submap::Handle handle_merged = map.merge_all_submaps();
    map.reconstruct("merged.ply", handle_merged);
}
int main() {
    // do_sphere_thing();
    // double_sphere_thing();

    std::vector<glm::vec3> points { 1'000'000 };
    std::random_device rd;
    std::mt19937 gen(420);
    std::uniform_real_distribution<double> dis(-1.0f, 1.0f);
    for (auto& point: points) {
        glm::dvec3 pointd = {
            dis(gen),
            dis(gen),
            dis(gen),
        };
        pointd = glm::normalize(pointd);
        pointd *= 5.0;
        point = (glm::vec3)pointd;
    }

    auto points_copy = points;
    glm::vec3 pos{ 99.2465, 99.1231, 99.3564 };
    // glm::vec3 pos{ 0, 0, 0 };
    for (auto& point: points_copy) point += pos;

    // build descriptors
    ndd::Descriptor desc{ points, glm::vec3{ 0, 0, 0 }};
    ndd::Descriptor desc_copy{ points_copy, pos };
    // check if loop closure is found
    std::vector<ndd::Descriptor> descriptors{ desc, desc_copy };
    std::vector<ndd::Descriptor::LookupKey> lookup_keys{ desc.get_lookup_key(), desc_copy.get_lookup_key() };
    ndd::detect_loop_closure(descriptors, lookup_keys, descriptors.size() - 1);

    // NDDManager manager;
    // manager.makeAndSaveNDDAndKeys(points);
    // manager.makeAndSaveNDDAndKeys(points_copy);
    // [[maybe_unused]] auto [a, b] = manager.detectLoopClosureID();

    return 0;
}
