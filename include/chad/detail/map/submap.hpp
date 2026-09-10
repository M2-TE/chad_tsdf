#pragma once
#include "chad/detail/dag/node.hpp"
#include "chad/detail/misc/pose.hpp"
#include "chad/detail/map/active_submap.hpp"

namespace chad::detail::map {
    struct Submap {
        // _roots needs to be set separately
        Submap(const std::vector<Pose>& poses, const std::vector<SubSubmap>& sub_submaps):
            _poses(poses),
            _sub_submaps(sub_submaps),
            _pose_err() {
            // average out all poses to get the submap "center"
            glm::aligned_dvec3 sum;
            for (const auto& pose: _poses) {
                sum += pose._position;
            }
            _pose_avg._position = sum / double(_poses.size());
        }

        dag::Addresses _roots; // root addresses for octrees in dag::Storage
        std::vector<Pose> _poses;
        std::vector<SubSubmap> _sub_submaps;
        Pose _pose_avg; // the pose average from all inserted scans
        Pose _pose_err; // pose error obtained from pose graph optimization
    };
}
