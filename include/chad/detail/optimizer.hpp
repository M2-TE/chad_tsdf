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
        void detect_loop_closure(SubmapIndex submap_i) {
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

                    // figure out submap index of the descriptor (TODO: just add a var for this in descriptors/poses)
                    bool submap_found = false;
                    SubmapIndex submap_other_i = 0;
                    for (uint32_t i = 0; i < uint32_t(_submaps.size()); i++) {
                        if (max_candidate >= _submaps[i]._scan_beg && max_candidate < _submaps[i]._scan_end) {
                            submap_other_i = i;
                            submap_found = true;
                            break;
                        }
                    }

                    // DEBUG: this is really just an assert atm
                    if (!submap_found) {
                        fmt::println("CORRESPONDING SUBMAP NOT FOUND");
                        std::exit(0);
                    }

                    // add new constraint to gtsam as per loop closure (TODO: needs more accurate tsdf to tsdf matching first!)
                    gtsam::Pose3 loop_measurement{ gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3{ 0, 0, 0 } };
                    gtsam::NonlinearFactorGraph factors;
                    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(submap_i), gtsam::symbol_shorthand::X(submap_other_i), loop_measurement, _loop_noise));
                    _isam.update(factors);

                }
                else fmt::println("loop not found (corr {:.2f})", max_correlation);
            }
        }

        // adds a finalized submap
        auto add_submap(RootIndices roots, ScanIndex scan_beg, ScanIndex scan_end) -> const Submap& {
            // avg of positions as submap center
            glm::dvec3 position{ 0, 0, 0 };
            for (ScanIndex scan_i = scan_beg; scan_i < scan_end; scan_i++) {
                const Pose& pose = _scan_poses[scan_i];
                position += glm::dvec3(pose._position);
            }
            position /= float(scan_end - scan_beg);

            // go ahead and create submap based on avg pose
            Submap submap{
                Pose{ position, glm::identity<glm::quat>() },
                Pose{}, // error
                roots,
                scan_beg,
                scan_end,
            };

            // add pose to gtsam
            using gtsam::symbol_shorthand::X;
            gtsam::Values values;
            gtsam::NonlinearFactorGraph factors;
            if (_submaps.size() == 0) {
                // anchor first submap pose
                gtsam::Pose3 prior_pose(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(position.x, position.y, position.z));
                factors.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prior_pose, _prior_noise));
                values.insert(X(0), prior_pose);
                _isam.update(factors, values);
            }
            else {
                const Pose& pose_prev = _submaps.back()._pose_avg;
                const Pose& pose_curr = submap._pose_avg;

                // add new pose to pose graph
                gtsam::Pose3 pose{ gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(position.x, position.y, position.z) };
                values.insert(X(_submaps.size()), pose);

                // add delta to previous pose as new factor
                const glm::vec3 pose_delta = pose_curr._position - pose_prev._position;
                gtsam::Pose3 pose_delta_gtsam{ gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(pose_delta.x, pose_delta.y, pose_delta.z) };
                factors.add(gtsam::BetweenFactor<gtsam::Pose3>{ X(_submaps.size() - 1), X(_submaps.size()), pose_delta_gtsam, _odom_noise });

                _isam.update(factors, values);
            }

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

        // GTSAM pose graph
        gtsam::SharedDiagonal _prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished());
        gtsam::SharedDiagonal _odom_noise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.2, 0.2, 0.2).finished());
        gtsam::SharedDiagonal _loop_noise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.02, 0.02, 0.02, 0.1, 0.1, 0.1).finished());
        static constexpr int _relinearize_skip = 1;
        static constexpr float _relinearize_threshold = 0.01;
        gtsam::ISAM2 _isam{ gtsam::ISAM2Params{ gtsam::ISAM2GaussNewtonParams(), _relinearize_threshold, _relinearize_skip }};
    };
}