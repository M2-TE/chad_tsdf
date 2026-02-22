#pragma once
#include <chad/indices.hpp>
#include "chad/detail/ndd.hpp"
#include "chad/detail/submap.hpp"
#include "chad/detail/nanoflann/KDTreeVectorOfVectorsAdaptor.hpp"

namespace chad::detail {
    struct MapOptimizer {
        MapOptimizer() = default;
        ~MapOptimizer() = default;

        // adds a new scan and creates a descriptor + lookup key for it
        void add_scan_descriptor(const std::vector<glm::vec3>& points, const Pose& pose) {
            _scan_poses.push_back(pose);
            _scan_descriptors.emplace_back(points, pose._position);
            _scan_lookup_keys.push_back(_scan_descriptors.back().get_lookup_key());
        }

        // find loop closure for multiple descriptors of same submap, returning pose error estimate if found
        auto detect_loop_closure(SubmapIndex submap_i) -> Pose {
            using namespace ndd;

            // indices for descriptors are within submap
            Submap& submap = _submaps[submap_i];
            const uint32_t descriptor_beg = submap._scan_beg;
            const uint32_t descriptor_end = submap._scan_end;

            // construct full KD tree with given keys
            auto tree = KDTreeVectorOfVectorsAdaptor<decltype(_scan_lookup_keys), float>{ Descriptor::N_RINGS, _scan_lookup_keys, 10 };

            // set up knn search within tree
            const uint32_t max_matches = 20 + descriptor_end - descriptor_beg; // allow finding matches with descriptors of same submap (will end up ignoring them)
            const uint32_t candidate_n = std::min<uint32_t>(max_matches, _scan_lookup_keys.size());


            for (uint32_t descriptor_i = descriptor_beg; descriptor_i < descriptor_end; descriptor_i++) {
                const Descriptor::LookupKey& key = _scan_lookup_keys[descriptor_i];
                const Descriptor& descriptor = _scan_descriptors[descriptor_i];

                // clean up storage for knnsearch_result
                nanoflann::KNNResultSet<float> knnsearch_result(candidate_n);
                std::vector<std::size_t> candidate_indices(candidate_n);
                std::vector<float> out_dists_sqr(candidate_n);
                knnsearch_result.init(candidate_indices.data(), out_dists_sqr.data());

                // find neighbours for the current descriptor key
                tree.index->findNeighbors(knnsearch_result, key.data(), nanoflann::SearchParameters(10));

                // find descriptor with highest correlation
                double max_correlation = 0.0;
                uint32_t max_candidate = 0;
                uint32_t max_shift = 0;
                for (const auto& candidate_i: candidate_indices) {
                    // ignore matches with own submap descriptors
                    if (candidate_i >= descriptor_beg && candidate_i < descriptor_end) continue;

                    const Descriptor& candidate = _scan_descriptors[candidate_i];
                    const auto [correlation, shift] = descriptor.estimate_correlation(candidate);
                    if (correlation > max_correlation) {
                        max_correlation = correlation;
                        max_candidate = candidate_i;
                        max_shift = shift;
                    }
                }

                // threshhold for correlation to count as a valid loop closure
                constexpr static double CORRELATION_THRESHHOLD = 0.65;
                if (max_correlation > CORRELATION_THRESHHOLD) {
                    static constexpr double SECTOR_ANGLE = 360.0 / double(Descriptor::N_SECTORS);
                    const float angle_degr = double(max_shift) * SECTOR_ANGLE;
                    // use pose delta as initial error estimate
                    const Pose& candidate_pose = _scan_poses[max_candidate];
                    const Pose& descriptor_pose = _scan_poses[descriptor_i];
                    Pose error = {
                        descriptor_pose._position - candidate_pose._position,
                        glm::quat(glm::vec3(0, glm::radians(angle_degr), 0))
                    };
                    fmt::println("loop found: NDD_{} matches NDD_{}. Error of ({:.2f},{:.2f},{:.2f}) with {:.2f}° yaw (corr {:.2f})",
                        descriptor_i, max_candidate,
                        error._position.x, error._position.y, error._position.z,
                        angle_degr, max_correlation);
                    return error; // TODO: should average found matches instead?
                }
                else fmt::println("loop not found (corr {:.2f})", max_correlation);
            }
            return {};
        }

        // adds a finalized submap
        auto add_submap(RootIndices roots, ScanIndex scan_beg, ScanIndex scan_end) -> const Submap& {
            // avg of positions as submap center
            glm::dvec3 position;
            for (ScanIndex scan_i = scan_beg; scan_i < scan_end; scan_i++) {
                const Pose& pose = _scan_poses[scan_i];
                position += glm::dvec3(pose._position);
            }
            position /= float(scan_end - scan_beg);

            // just go ahead and create submap
            Submap submap{
                Pose{ glm::vec3(position), glm::quat() },
                Pose{},
                roots,
                scan_beg,
                scan_end,
            };
            _submaps.push_back(submap);
            return _submaps.back();
        }

        // check if the current active submap has crossed the position delta threshhold
        bool is_active_submap_done(const Pose& pose_new, ScanIndex submap_beg, float threshhold) {
            const Pose& pose_prev = _scan_poses[submap_beg];
            float distance = glm::distance(pose_prev._position, pose_new._position);
            // if our submap threshhold is crossed, finalize the active submap before inserting new points
            if (distance > threshhold) return true;
            else return false;
        }

        // persistent data per submap
        std::vector<Submap>      _submaps;
        std::vector<SubmapIndex> _merged_submaps;

        // persistent data per scan
        std::vector<Pose>                       _scan_poses;
        std::vector<ndd::Descriptor>            _scan_descriptors;
        std::vector<ndd::Descriptor::LookupKey> _scan_lookup_keys;

        // keep track of current global pose to transform new incoming pointclouds
        Pose _global_pose;
    };
}