#include "chad/tsdf.hpp"

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
    map.save("mesh.ply");
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
            dis(gen)
        };
        pointd = glm::normalize(pointd);
        pointd *= 5.0;
        point = (glm::vec3)pointd;
        point += position;
    }
    map.insert(points, position);
    chad::Submap sub_0 = map.finalize();

    // insert second sphere into CHAD TSDF
    position = {3.63456, 3.90122, 3.01233};

    for (auto& point: points) {
        glm::dvec3 pointd = {
            dis(gen),
            dis(gen),
            dis(gen)
        };
        pointd = glm::normalize(pointd);
        pointd *= 5.0;
        point = (glm::vec3)pointd;
        point += position;
    }
    map.insert(points, position);
    chad::Submap sub_1 = map.finalize();

    // try matching
    chad::Submap sub_merged = map.merge_submaps(sub_0, sub_1);
    // map.save("raw_0.ply", sub_0);
    // map.save("raw_1.ply", sub_1);
    map.save("mergetest.ply", sub_merged);
}
int main() {
    // do_sphere_thing();
    double_sphere_thing();
}