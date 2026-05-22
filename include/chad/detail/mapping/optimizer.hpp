#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/mapping/submap.hpp"
#include "chad/detail/mapping/indices.hpp"
#include "chad/detail/mapping/active_submap.hpp"

namespace chad::detail::mapping {
    struct Optimizer {
        Optimizer(float submap_xyz_threshhold, float submap_cor_threshhold);
        ~Optimizer();

        void add_scan(const std::vector<glm::aligned_vec3>& points, const std::vector<glm::aligned_vec3>& normals, const ndd::Descriptor& descriptor, Pose pose) {
            // 1. check whether translational delta threshhold was crossed
            bool crossed_TODO = false;
            if (!_active_submap._scan_indices.empty()) {
                Pose pose_first = _scan_poses[_active_submap._scan_indices.front()];
                float distance = glm::distance(pose_first._position, pose._position);
                if (distance > _submap_xyz_threshhold) crossed_TODO = true;
            }
            // TODO: all the stuff when submap needs to be finalized

            // 2. check whether NDD correlation threshhold was crossed

        }

        // adds a new scan and creates a descriptor + lookup key for it
        [[deprecated]] void add_scan_descriptor(const std::vector<glm::aligned_vec3>& points, const Pose& pose);
        // find loop closure for multiple descriptors of same submap, returning pose error estimate if found
        [[deprecated]] void detect_loop_closure(SubmapIndex submap_i);
        // adds a finalized submap
        [[deprecated]] auto add_submap(dag::RootIndices roots, ScanIndex scan_beg, ScanIndex scan_end) -> const Submap&;
        // check if the current active submap has crossed the position delta threshhold
        [[deprecated]] bool is_active_submap_done(const Pose& pose_new, ScanIndex submap_beg, float threshhold);
        // temporary
        [[deprecated]] void debug_thingy();

        // settings
        const float _submap_xyz_threshhold;
        const float _submap_cor_threshhold;
        // transient data during submapping
        ActiveSubmap _active_submap;
        // persistent data per submap
        std::vector<Submap> _submaps;
        // persistent data per scan
        std::vector<Pose>                       _scan_poses;
        std::vector<SubmapIndex>                _scan_submap; // for easier association
        std::vector<ndd::Descriptor>            _scan_descriptors;
        std::vector<ndd::Descriptor::LookupKey> _scan_lookup_keys;
        // persistent data per pose graph
        std::unique_ptr<struct GTSAMData> _gtsam;

        // OLD
        [[deprecated]] std::vector<SubmapIndex> _merged_submaps;
    };
}
