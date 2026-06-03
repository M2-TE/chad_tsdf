#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/funcs/timing.hpp"
#include "chad/detail/mapping/submap.hpp"
#include "chad/detail/mapping/indices.hpp"
#include "chad/detail/mapping/active_submap.hpp"

namespace chad::detail::mapping {
    struct Optimizer {
        Optimizer(float sdf_res, float sdf_trunc, float submap_xyz_threshhold, float submap_cor_threshhold);
        ~Optimizer();

        void add_scan(std::vector<glm::aligned_vec3>&& points, std::vector<glm::aligned_vec3>&& normals, Pose pose, ndd::Descriptor& descriptor, std::jthread& descriptor_thread) {
            // Submap: check whether translational delta threshhold was crossed
            if (!_active_submap._all_poses.empty()) {
                auto timestamp = std::chrono::steady_clock::now();
                Pose pose_first = _active_submap._all_poses.front();
                float distance = glm::distance(pose_first._position, pose._position);
                if (distance > _submap_xyz_threshhold) {
                    on_submap_completion();
                    MEASURE_TIME(timestamp, "\t-> submap completed");
                }
            }

            // wait for the descriptor construction to finish
            auto timestamp = std::chrono::steady_clock::now();
            descriptor_thread.join();
            MEASURE_TIME(timestamp, "\t-> waited for descriptor construction to finish");

            // Sub-Submap: check whether NDD correlation threshhold was crossed
            if (!_active_submap._sub_poses.empty()) {
                auto timestamp = std::chrono::steady_clock::now();
                const auto& descriptor_latest = _descriptors[_active_submap._descriptor_indices.back()];
                const auto [correlation, rotation] = descriptor_latest.estimate_correlation(descriptor);
                if (correlation < _submap_cor_threshhold) {
                    on_sub_submap_completion(std::move(descriptor), pose);
                    MEASURE_TIME(timestamp, "\t-> sub-submap completion");
                }
            }
            // when current sub-submap is empty, initialize it
            else {
                auto timestamp = std::chrono::steady_clock::now();
                on_sub_submap_completion(std::move(descriptor), pose);
                MEASURE_TIME(timestamp, "\t-> sub-submap initialization");
            }

            // insert new data into active submap
            timestamp = std::chrono::steady_clock::now();
            _active_submap.add_frame(std::move(points), std::move(normals), pose);
            MEASURE_TIME(timestamp, "\t-> added scan frame to active submap");

            // DEBUG!!!!
            timestamp = std::chrono::steady_clock::now();
            on_sub_submap_completion(std::move(descriptor), pose);
            MEASURE_TIME(timestamp, "\t-> DEBUG(TODO async): added scan frame to active submap");
        }

    private:
        // finish entire submap and create DAG octree
        void on_submap_completion() {
            // TODO: finalize as DAG octree
            // TODO: gotta consider sub-submapping cases?

            fmt::println("TODO: submap finish");

            // clean up
            _active_submap.clear();
        }

        // TODO: LOOP CLOSURE HERE! -> only need to update the lookup tree after every ~5 (or all above threshhold) descriptor insertions (based on how many prev ones to skip)
        // TODO: use point-to-tsdf for more accurate err estimation after loop closure
        // TODO: store the best few candidates for matches and find the best ones once ENTIRE SUBMAP is about to be finished
        // -> relying on single sub-submap to sub-submap matches would be too unreliable

        // finish only the sub-submap
        void on_sub_submap_completion(ndd::Descriptor&& descriptor, Pose pose) {
            // update TSDF octree and clear out all sub-submap data
            _active_submap.write_octree<double>(pose, _sdf_res, _sdf_trunc);
            // set up new sub-submap
            _active_submap._descriptor_indices.push_back(_descriptors.size());
            _lookup_keys.push_back(descriptor.get_lookup_key());
            _descriptors.push_back(std::move(descriptor));
        }

    public:
        // settings
        const float _sdf_res;   // copied from TSDFMap::_sdf_res
        const float _sdf_trunc; // copied from TSDFMap::_sdf_trunc
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
