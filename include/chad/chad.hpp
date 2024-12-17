#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include <span>
#include "chad/subtree.hpp"

// TODO doxygen comments
// TODO retrieve subtree based on pos

struct Chad {
    Chad();

    /**
     * @brief Main function to insert new points
     * 
     * @param points array of 3D points
     * @param normals array of 3D normals
     * @param position scanner position
     * @param rotation scanner rotation
     */
    void insert(std::span<std::array<float, 3>> points, std::span<std::array<float, 3>> normals, std::array<float, 3> position, std::array<float, 4> rotation);
    /**
     * @brief Main function to insert new points
     * 
     * @param points array of 3D points
     * @param position scanner position
     * @param rotation scanner rotation
     */
    template<typename PosArr, typename RotArr>
    void insert(std::span<std::array<float, 3>> points, PosArr position, RotArr rotation) {
        // interpret the position and rotation as 3/4 piece vectors
        std::array<float, 3>* pos_p = reinterpret_cast<std::array<float, 3>*>(&position);
        std::array<float, 4>* rot_p = reinterpret_cast<std::array<float, 4>*>(&rotation);
        // 
        insert(points, {}, *pos_p, *rot_p);
    }
    template<typename PosArr, typename RotArr>
    void insert(std::span<std::array<float, 3>> points, std::span<std::array<float, 3>> normals, PosArr position, RotArr rotation) {
        // interpret the position and rotation as 3/4 piece vectors
        std::array<float, 3>* pos_p = reinterpret_cast<std::array<float, 3>*>(&position);
        std::array<float, 4>* rot_p = reinterpret_cast<std::array<float, 4>*>(&rotation);
        // 
        insert(points, normals, *pos_p, *rot_p);
    }
    /**
     * @brief Merge target subtree into the global map
     * 
     * @param root_addr root address of the target subtree
     */
    void merge_subtree(uint_fast32_t root_addr);

    /**
     * @brief Merge all subtrees into the global map
     */
    void merge_all_subtrees();

    /**
     * @brief Print statistics into the console
     */
    void print_stats();

    /**
     * @brief Get the memory footprint of the node vectors
     * 
     * @return memory footprint in bytes
     */
    auto get_readonly_size() -> double;

    /**
     * @brief Get the memory footprint of the hash structures
     * 
     * @return memory footprint in bytes
     */
    auto get_hash_size() -> double;

    /**
     * @brief Ignore this one
     */
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