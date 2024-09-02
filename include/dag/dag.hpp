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
    void merge_primary(uint_fast32_t root_addr);
    void reconstruct();
    void print_stats();

private:
    auto insert_octree(struct Octree& octree, std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals)
        -> uint32_t;

private:
    std::array<NodeLevel, 63/3 - 1> _node_levels;
    LeafLevel _leaf_level;
};