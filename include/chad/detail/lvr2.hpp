#pragma once
#include "chad/indices.hpp"
#include "chad/detail/dag_storage.hpp"

namespace chad::detail {
    void reconstruct(const DAGStorage& dag, const RootIndices& roots, float voxel_res, float trunc_dist, std::string_view filename, const std::array<uint8_t, 3>& col);
}