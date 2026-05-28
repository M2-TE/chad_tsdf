#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/mapping/submap.hpp"
#include "chad/detail/mapping/indices.hpp"
#include "chad/detail/mapping/active_submap.hpp"

namespace chad::detail::mapping {
    struct Optimizer {
        Optimizer(float submap_xyz_threshhold, float submap_cor_threshhold);
        ~Optimizer();

        void add_scan(const std::vector<glm::aligned_vec3>& points, const std::vector<glm::aligned_vec3>& normals, ndd::Descriptor&& descriptor, Pose pose) {
            // Submap: check whether translational delta threshhold was crossed
            if (!_active_submap._all_poses.empty()) {
                Pose pose_first = _active_submap._all_poses.front();
                float distance = glm::distance(pose_first._position, pose._position);
                if (distance > _submap_xyz_threshhold) on_submap_finish();
            }

            // Sub-Submap: check whether NDD correlation threshhold was crossed
            if (!_active_submap._all_poses.empty()) {
                const auto& descriptor_latest = _descriptors[_active_submap._descriptor_indices.back()];
                const auto [correlation, rotation] = descriptor_latest.estimate_correlation(descriptor);
                if (correlation < _submap_cor_threshhold) on_sub_submap_finish(std::move(descriptor));
            }

            // insert new data into active submap
            _active_submap.add_data(points, normals, pose);
        }

    private:
        // finish entire submap and create DAG octree
        void on_submap_finish() {
            // TODO: finalize as DAG octree

            fmt::println("TODO: submap finish");


            // clean up
            _active_submap.clear_all();
        }
        // finish only the sub-submap
        void on_sub_submap_finish(ndd::Descriptor&& descriptor) {
            fmt::println("TODO: sub-submap finish (maybe?)");
            _descriptors.push_back(descriptor);
            _lookup_keys.push_back(descriptor.get_lookup_key());
            // clean up
            _active_submap.clear_sub();
        }

    public:
        // settings
        const float _submap_xyz_threshhold;
        const float _submap_cor_threshhold;
        // transient data during submapping
        ActiveSubmap _active_submap;
        // persistent data per submap
        std::vector<Submap> _submaps;
        // persistent data per sub-submap
        std::vector<ndd::Descriptor>            _descriptors;
        std::vector<ndd::Descriptor::LookupKey> _lookup_keys;
        // persistent data for pose graph
        std::unique_ptr<struct GTSAMData> _gtsam;

        // OLD
        [[deprecated]] std::vector<SubmapIndex> _merged_submaps;
        [[deprecated]] std::vector<Pose>        _scan_poses;
        [[deprecated]] std::vector<SubmapIndex> _scan_submap; // for easier association
    };
}
