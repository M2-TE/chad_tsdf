#pragma once
#include <string_view>
#include "chad/detail/dag.hpp"
#include "chad/detail/submap.hpp"

namespace chad::detail {
    void reconstruct(const DAG& dag, const detail::Submap& submap, float voxel_res, float trunc_dist, std::string_view filename);
}