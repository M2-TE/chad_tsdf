#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include "dag/subtree.hpp"

#if __has_include(<Eigen/Eigen>)
    #include <span>
    #include <Eigen/Eigen>
    #define CHAD_EIGEN
#endif

// TODO doxygen comments
// TODO retrieve subtree based on pos

struct DAG {
    DAG();
    
    #ifdef CHAD_EIGEN
    void insert(std::span<Eigen::Vector3f> points, Eigen::Vector3f position, Eigen::Quaternionf rotation) {
        std::array<float, 3>* pos_p = reinterpret_cast<std::array<float, 3>*>(&position);
        std::array<float, 4>* rot_p = reinterpret_cast<std::array<float, 4>*>(&rotation);
        insert(reinterpret_cast<std::array<float, 3>*>(points.data()), points.size(), *pos_p, *rot_p);
    }
    #endif

    /**
     * @brief Main function to insert new points
     * 
     * @param points_p array of 3D points
     * @param points_count number of points
     * @param position scanner position
     * @param rotation scanner rotation
     */
    void insert(std::array<float, 3>* points_p, std::size_t points_count, std::array<float, 3> position, std::array<float, 4> rotation);
    /**
     * @brief Main function to insert new points
     * 
     * @param points_p array of 3D points
     * @param points_count number of points
     * @param position scanner position
     * @param rotation scanner rotation
     */
    template<typename PosArr, typename RotArr>
    void insert(std::vector<PosArr>& points, PosArr position, RotArr rotation) {
        std::array<float, 3>* pos_p = reinterpret_cast<std::array<float, 3>*>(&position);
        std::array<float, 4>* rot_p = reinterpret_cast<std::array<float, 4>*>(&rotation);
        insert(reinterpret_cast<std::array<float, 3>*>(points.data()), points.size(), *pos_p, *rot_p);
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