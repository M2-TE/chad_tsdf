#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include "dag/subtree.hpp"

// TODO doxygen comments
// TODO retrieve subtree based on pos

struct DAG {
    DAG();
    // helper function to insert points into DAG
    template<typename PosArr, typename RotArr>
    void insert(std::vector<PosArr>& points, PosArr position, RotArr rotation) {
        std::array<float, 3>* pos_p = reinterpret_cast<std::array<float, 3>*>(&position);
        std::array<float, 4>* rot_p = reinterpret_cast<std::array<float, 4>*>(&rotation);
        insert(reinterpret_cast<std::array<float, 3>*>(points.data()), points.size(), *pos_p, *rot_p);
    }
    // main function to insert points into DAG'


    /**
     * @brief 
     * 
     * @param points_p 
     * @param points_count 
     * @param position 
     * @param rotation 
     */
    void insert(std::array<float, 3>* points_p, std::size_t points_count, std::array<float, 3> position, std::array<float, 4> rotation);
    // TODO: insert via 4x4 mat as pose
    // TODO: rotation as euler angles
    void merge_subtree(uint_fast32_t root_addr);
    void merge_all_subtrees();
    void print_stats();
    auto get_readonly_size() -> double;
    auto get_hash_size() -> double;
    auto debug_iterate_all_leaves_of_subtree(uint32_t root_addr) -> std::size_t;

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

public:
    std::vector<Subtree> _subtrees;
private:
    std::array<struct NodeLevel, 63/3 - 1>* const _node_levels_p;
    struct LeafLevel* const _leaf_level_p;
};