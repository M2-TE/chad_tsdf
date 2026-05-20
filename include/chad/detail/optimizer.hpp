#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/dag/submap.hpp"

namespace chad::detail {
    struct GTSAMData;
    struct MapOptimizer {
        MapOptimizer();
        ~MapOptimizer();

        // adds a new scan and creates a descriptor + lookup key for it
        void add_scan_descriptor(const std::vector<glm::aligned_vec3>& points, const Pose& pose);
        // find loop closure for multiple descriptors of same submap, returning pose error estimate if found
        void detect_loop_closure(dag::Submap::Index submap_i);
        // adds a finalized submap
        auto add_submap(dag::RootIndices roots, ScanIndex scan_beg, ScanIndex scan_end) -> const dag::Submap&;
        // check if the current active submap has crossed the position delta threshhold
        bool is_active_submap_done(const Pose& pose_new, ScanIndex submap_beg, float threshhold);

        // temporary
        void debug_thingy();

        // persistent data per submap
        std::vector<dag::Submap>        _submaps;
        std::vector<dag::Submap::Index> _merged_submaps;

        // persistent data per scan
        std::vector<Pose>                       _scan_poses;
        std::vector<dag::Submap::Index>         _scan_submap; // for easier association
        std::vector<ndd::Descriptor>            _scan_descriptors;
        std::vector<ndd::Descriptor::LookupKey> _scan_lookup_keys;

        // GTSAM pose graph things (forward declared)
        std::unique_ptr<struct GTSAMData> _gtsam;
    };
}
