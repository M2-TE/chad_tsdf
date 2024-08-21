#pragma once
#include "glm/fwd.hpp"
#pragma once
#include <array>
#include <vector>
#include <glm/glm.hpp> // TODO: forward declare this stuff
#include <glm/gtc/quaternion.hpp>
#include "dag/levels.hpp"

class DAG {
public:
    DAG();
    void insert(std::vector<glm::vec3>& points, glm::vec3 position, glm::quat rotation);
    void reconstruct();
    void print_stats();

private:
    static constexpr std::size_t _level_count = 21; // TODO: standardize this better alongside normal levels + leaf level
    std::array<NodeLevel, _level_count> _node_levels;
    LeafLevel _leaf_level;
};