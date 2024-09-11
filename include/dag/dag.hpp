#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include "dag/subtree.hpp"

struct DAG {
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
    void print_stats();

    // get raw data from normal dag levels
    auto get_node_levels() -> std::array<std::vector<uint32_t>*, 63/3 - 1>;
#if LEAF_BITS == 8
    // get raw data from leaf dag level
    auto get_leaf_level() -> std::vector<uint64_t>&;
#elif LEAF_BITS == 4
    // get raw data from leaf dag level
    auto get_leaf_level() -> std::vector<uint32_t>&;
#endif
    // get voxel resolution
    auto get_voxel_resolution() -> double;

private:
    std::vector<Subtree> _subtrees;
    std::array<struct NodeLevel, 63/3 - 1>* const _node_levels_p;
    struct LeafLevel* const _leaf_level_p;
};