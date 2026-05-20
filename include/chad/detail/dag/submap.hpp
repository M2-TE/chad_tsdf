#pragma once
#include "chad/detail/pose.hpp"
#include "chad/detail/scan_index_RENAME.hpp"
#include "chad/detail/dag/root_indices.hpp"

namespace chad::detail::dag {
    struct Submap {
        using Index = std::uint32_t;

        RootIndices _root_indices; // roots for octrees in dag::Storage
        ScanIndex _scan_beg; // index of first scan
        ScanIndex _scan_end; // past-the-end index of final scan
        Pose _pose_avg; // the pose average from all inserted scans
        Pose _pose_err; // pose error obtained from pose graph optimization
    };
}
