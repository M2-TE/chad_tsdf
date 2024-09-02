#pragma once
#include <span>
#include <array>
#include <vector>

class DAG {
public:
    DAG();
    // helper function to insert points into DAG
    template<typename PosArr, typename RotArr>
    void insert(std::span<PosArr> points, PosArr position, RotArr rotation) {
        std::array<float, 3>* pos_p = reinterpret_cast<std::array<float, 3>*>(&position);
        std::array<float, 4>* rot_p = reinterpret_cast<std::array<float, 4>*>(&rotation);
        insert(reinterpret_cast<std::array<float, 3>*>(points.data()), points.size(), *pos_p, *rot_p);
    }
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

private:
    std::array<struct NodeLevel, 63/3 - 1>* _node_levels_p;
    struct LeafLevel* _leaf_level_p;
};