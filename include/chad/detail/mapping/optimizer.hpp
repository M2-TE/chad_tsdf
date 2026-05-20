#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/mapping/submap.hpp"
#include "chad/detail/mapping/indices.hpp"
#include "chad/detail/mapping/active_submap.hpp"

namespace chad::detail::mapping {
    struct Optimizer {
        Optimizer();
        ~Optimizer();

        // adds a new scan and creates a descriptor + lookup key for it
        void add_scan_descriptor(const std::vector<glm::aligned_vec3>& points, const Pose& pose);
        // find loop closure for multiple descriptors of same submap, returning pose error estimate if found
        void detect_loop_closure(SubmapIndex submap_i);
        // adds a finalized submap
        auto add_submap(dag::RootIndices roots, ScanIndex scan_beg, ScanIndex scan_end) -> const Submap&;
        // check if the current active submap has crossed the position delta threshhold
        bool is_active_submap_done(const Pose& pose_new, ScanIndex submap_beg, float threshhold);

        // temporary
        [[deprecated]] void debug_thingy();

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
