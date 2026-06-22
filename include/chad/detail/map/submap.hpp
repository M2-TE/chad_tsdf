#pragma once
#include "chad/detail/pose.hpp"
#include "chad/detail/map/indices.hpp"
#include "chad/detail/dag/root_indices.hpp"

namespace chad::detail::map {
    struct Submap {
        dag::RootIndices _root_indices; // roots for octrees in dag::Storage
        DescriptorIndex _ndd_beg; // iterator for Optimizer::_descriptors
        DescriptorIndex _ndd_end; // iterator for Optimizer::_descriptors
        std::vector<Pose> _poses;
        Pose _pose_avg; // the pose average from all inserted scans
        Pose _pose_err; // pose error obtained from pose graph optimization

        // TODO: store all the NDDs in here too
        // TODO: set of indices into ndd vector as the "most important" onces (during sub-sub-mapping)
    };
}
