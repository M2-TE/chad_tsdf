#pragma once
#include "glm/fwd.hpp"
#pragma once
#include <array>
#include <vector>

class DAG {
public:
    DAG();
    // helper function to insert points into DAG
    template<typename PosArr, typename RotArr>
    void insert(std::vector<PosArr>& points, PosArr position, RotArr rotation) {
        std::array<float, 3>* pos_p = reinterpret_cast<std::array<float, 3>*>(&position);
        std::array<float, 4>* rot_p = reinterpret_cast<std::array<float, 4>*>(&rotation);
        insert(reinterpret_cast<std::array<float, 3>*>(points.data()), points.size(), *pos_p, *rot_p);
    }
    // main function to insert points into DAG
    void insert(std::array<float, 3>* points_p, std::size_t points_count, std::array<float, 3> position, std::array<float, 4> rotation);
    void merge_primary(uint_fast32_t root_addr);
    void reconstruct();
    void print_stats();

private:
    auto insert_octree(struct Octree& octree, std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals) -> uint32_t;

private:
    std::array<struct NodeLevel, 63/3 - 1>* _node_levels_p;
    struct LeafLevel* _leaf_level_p;
};