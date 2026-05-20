#pragma once
#include "chad/indices.hpp"
#include "chad/detail/pose.hpp"

namespace chad::detail {
    struct Submap {
        Submap(const Pose& pose_avg,
                const Pose& pose_err,
                const RootIndices& root_indices,
                ScanIndex scan_beg,
                ScanIndex scan_end):
            _pose_avg(pose_avg),
            _pose_err(pose_err),
            _root_indices(root_indices),
            _scan_beg(scan_beg),
            _scan_end(scan_end) {
        }

        Pose _pose_avg;
        Pose _pose_err;
        RootIndices _root_indices;
        ScanIndex _scan_beg; // index of first scan
        ScanIndex _scan_end; // past-the-end index of scan
    };
};
