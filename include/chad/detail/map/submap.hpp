#pragma once
#include "chad/detail/pose.hpp"
#include "chad/detail/dag/node.hpp"
#include "chad/detail/map/indices.hpp"
#include "chad/detail/map/active_submap.hpp"

namespace chad::detail::map {
    struct Submap {
        Submap(ActiveSubmap& active_submap) {
            // active submap will soon be discarded, so we can safely swap the pose data
            _poses.swap(active_submap._all_poses);
            // TODO
        }

        dag::Addresses _roots; // roots for octrees in dag::Storage
        std::vector<Pose>            _ndd_poses;
        std::vector<DescriptorIndex> _ndd_indices; // one ndd per sub-submap
        std::vector<Pose> _poses;
        Pose _pose_avg; // the pose average from all inserted scans
        Pose _pose_err; // pose error obtained from pose graph optimization

        // TODO: store all the NDDs in here too
        // TODO: set of indices into ndd vector as the "most important" onces (during sub-sub-mapping)
    };
}
