#pragma once
#include "chad/submap.hpp"
#include "chad/detail/dag.hpp"

namespace chad::detail {
    void reconstruct(const DAG& dag, const Submap& submap, float voxel_res, float trunc_dist, std::string_view filename);
}