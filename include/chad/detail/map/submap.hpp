#pragma once
#include "chad/detail/dag/node.hpp"
#include "chad/detail/misc/pose.hpp"
#include "chad/detail/map/indices.hpp"
#include "chad/detail/map/active_submap.hpp"

namespace chad::detail::map {
    struct Submap {
        Submap(ActiveSubmap& active_submap): _pose_err() {
            // active submap will soon be discarded, so we can safely swap the vectors
            _poses.swap(active_submap._all_poses);
            _descriptor_indices.swap(active_submap._descriptor_indices);

            // average out all poses to get the submap "center"
            // TODO: how the hell to handle rotations, averaging them would just be wrong in a lot of cases?
            glm::aligned_dvec3 sum;
            for (const auto& pose: active_submap._all_poses) {
                sum += pose._position;
            }
            _pose_avg._position = sum / double(active_submap._all_poses.size());

            // -> _roots will be updated separately
        }

        dag::Addresses _roots; // roots for octrees in dag::Storage
        std::vector<Pose> _poses;
        std::vector<DescriptorIndex> _descriptor_indices; // one ndd per sub-submap
        Pose _pose_avg; // the pose average from all inserted scans
        Pose _pose_err; // pose error obtained from pose graph optimization
    };
}
