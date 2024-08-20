#pragma once
#include "glm/fwd.hpp"
#include <array>
#include <vector>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class DAG {
public:
    DAG();
    ~DAG();
    void insert(std::vector<glm::vec3>& points, glm::vec3 position, glm::quat rotation);
    void reconstruct();
    void print_stats();

private:
    static constexpr std::size_t _level_count = 21; // TODO: standardize this better alongside normal levels + leaf level

    // stat and debugging trackers
    std::array<uint32_t, _level_count+1> uniques;
    std::array<uint32_t, _level_count+1> dupes;
    std::vector<glm::vec3> _all_points;
};