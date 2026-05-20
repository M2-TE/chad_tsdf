#pragma once
#include "chad/detail/pose.hpp"
#include "chad/detail/mapping/indices.hpp"
#include "chad/detail/dag/root_indices.hpp"

namespace chad::detail::mapping {
    struct Submap {
        dag::RootIndices _root_indices; // roots for octrees in dag::Storage
        ScanIndex _scan_beg; // index of first scan
        ScanIndex _scan_end; // past-the-end index of final scan
        Pose _pose_avg; // the pose average from all inserted scans
        Pose _pose_err; // pose error obtained from pose graph optimization

        // TODO: store all the NDDs in here too
        // TODO: set of indices into ndd vector as the "most important" onces (during sub-sub-mapping)
    };
}
