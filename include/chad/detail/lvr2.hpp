#pragma once
#include <string_view>
#include "chad/detail/levels.hpp"
#include "chad/detail/submap.hpp"

namespace chad::detail {
    void reconstruct(const detail::Submap& submap, const NodeLevels& node_levels, float voxel_res, float trunc_dist, std::string_view filename);
}