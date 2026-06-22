#include "chad/detail/map/optimizer.hpp"

// all the gtsam headers, mostly taken from their example
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// KD-Tree for finding NDD matches
#include "chad/detail/ndd/nanoflann/KDTreeVectorOfVectorsAdaptor.hpp"

namespace chad::detail::map {
    struct GTSAMData {
        static constexpr int _relinearize_skip = 1;
        static constexpr float _relinearize_threshold = 0.01;
        gtsam::SharedDiagonal _prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished());
        gtsam::SharedDiagonal _odom_noise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.2, 0.2, 0.2).finished());
        gtsam::SharedDiagonal _loop_noise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.02, 0.02, 0.02, 0.1, 0.1, 0.1).finished());
        gtsam::ISAM2 _isam{ gtsam::ISAM2Params{ gtsam::ISAM2GaussNewtonParams(), _relinearize_threshold, _relinearize_skip }};
    };
    Optimizer::Optimizer(float sdf_res, float sdf_trunc, float submap_xyz_threshhold, float submap_cor_threshhold):
        _sdf_res(sdf_res),
        _sdf_trunc(sdf_trunc),
        _submap_xyz_threshhold(submap_xyz_threshhold),
        _submap_cor_threshhold(submap_cor_threshhold) {
    }
    Optimizer::~Optimizer() {
        // wait for all threads to finish their work before exiting
        for (auto& thread: _active_threads) {
            if (thread.joinable()) thread.join();
        }
    }

    // void Optimizer::add_scan_descriptor(const std::vector<glm::aligned_vec3>& points, const Pose& pose) {
    //     _scan_poses.push_back(pose);
    //     _scan_submap.push_back(_submaps.size());
    //     _scan_descriptors.emplace_back(points, pose._position);
    //     _scan_lookup_keys.push_back(_scan_descriptors.back().get_lookup_key());
    // }
    // void Optimizer::detect_loop_closure(SubmapIndex submap_i) {
    //     using namespace ndd;

    //     // indices for descriptors are within submap
    //     Submap& submap = _submaps[submap_i];
    //     const uint32_t descriptor_beg = submap._scan_beg;
    //     const uint32_t descriptor_end = submap._scan_end;

    //     // construct full KD tree with given keys
    //     auto tree = KDTreeVectorOfVectorsAdaptor<decltype(_scan_lookup_keys), float>{ Descriptor::N_RINGS, _scan_lookup_keys, 10 };

    //     // set up knn search within tree
    //     const uint32_t max_matches = 20 + descriptor_end - descriptor_beg; // allow finding matches with descriptors of same submap (will end up ignoring them)
    //     const uint32_t candidate_n = std::min<uint32_t>(max_matches, _scan_lookup_keys.size());

    //     // store each match above correlation threshhold
    //     struct Correlation {
    //         float confidence = 0.0f;
    //         uint32_t descriptor_self_i = 0;
    //         uint32_t descriptor_other_i = 0;
    //         uint32_t sector_shift = 0;
    //     };
    //     // we need to map correlations to their respective submap pairings
    //     std::map<SubmapIndex, std::vector<Correlation>> correlations;
    //     uint32_t correlation_count = 0;

    //     // for every descriptor within submap, try to find correlations with other submaps
    //     for (uint32_t descriptor_i = descriptor_beg; descriptor_i < descriptor_end; descriptor_i++) {
    //         const Descriptor::LookupKey& key = _scan_lookup_keys[descriptor_i];
    //         const Descriptor& descriptor = _scan_descriptors[descriptor_i];

    //         // clean up storage for knnsearch_result
    //         nanoflann::KNNResultSet<float> knnsearch_result(candidate_n);
    //         std::vector<std::size_t> candidate_indices(candidate_n);
    //         std::vector<float> out_dists_sqr(candidate_n);
    //         knnsearch_result.init(candidate_indices.data(), out_dists_sqr.data());

    //         // find neighbours for the current descriptor key
    //         tree.index->findNeighbors(knnsearch_result, key.data(), nanoflann::SearchParameters(10));

    //         // find descriptor with highest correlation
    //         double max_correlation = 0.0;
    //         uint32_t max_candidate = 0;
    //         uint32_t max_shift = 0;
    //         for (const auto& candidate_i: candidate_indices) {
    //             // ignore matches with own submap descriptors
    //             if (candidate_i >= descriptor_beg && candidate_i < descriptor_end) continue;

    //             const Descriptor& candidate = _scan_descriptors[candidate_i];
    //             const auto [correlation, shift] = descriptor.estimate_correlation(candidate);
    //             if (correlation > max_correlation) {
    //                 max_correlation = correlation;
    //                 max_candidate = candidate_i;
    //                 max_shift = shift;
    //             }
    //         }

    //         // threshhold for correlation to even be considered as a loop closure candidate
    //         constexpr static double CORRELATION_THRESHHOLD = 0.95; // TODO: move to TSDFMap as parameter
    //         if (max_correlation > CORRELATION_THRESHHOLD) {
    //             SubmapIndex index = _scan_submap[max_candidate];
    //             Correlation correlation {
    //                 float(max_correlation),
    //                 descriptor_i,
    //                 max_candidate,
    //                 max_shift,
    //             };
    //             // map the correlation to the correct submap
    //             auto [emplaced_it, emplaced_b] = correlations.try_emplace(index);
    //             emplaced_it->second.push_back(correlation);
    //             correlation_count++;
    //         }
    //     }

    //     // TODO: increase to 6 or something
    //     if (correlation_count < 6) {
    //         fmt::println("Insufficient total NDD correlations found ({})", correlation_count);
    //         return;
    //     }

    //     // each submap has its own GTSAM node, so need to be handled separately
    //     for (const auto& [submap_other_i, correlation_vector]: correlations) {

    //         // TODO: increase to 6 or something
    //         if (correlation_vector.size() < 3) {
    //             fmt::println("Insufficient NDD correlations with submap {} to proceed ({})", submap_other_i, correlation_count);
    //             return;
    //         }

    //         // calculate mean error in translation and rotation
    //         uint32_t mean_shift = 0;
    //         glm::dvec3 mean_translation{ 0, 0, 0 };
    //         for (const auto& correlation: correlation_vector) {
    //             const auto& pose_self = _scan_poses[correlation.descriptor_self_i];
    //             const auto& pose_other = _scan_poses[correlation.descriptor_other_i];
    //             mean_translation += glm::dvec3(pose_self._position - pose_other._position);
    //             mean_shift += correlation.sector_shift;
    //         }
    //         mean_translation /= double(correlation_vector.size());
    //         mean_shift /= correlation_vector.size();

    //         static constexpr double SECTOR_ANGLE = 360.0 / double(Descriptor::N_SECTORS);
    //         const float angle_degr = double(mean_shift) * SECTOR_ANGLE;
    //         Pose error{ glm::aligned_dvec3{ mean_translation }, glm::aligned_dvec3{ 0, angle_degr, 0 }};

    //         fmt::println("Detected loop closure with submap {} ({} correlations). Error of ({:.2f},{:.2f},{:.2f}) with {:.2f}° yaw",
    //             submap_other_i, correlation_vector.size(), error._position.x, error._position.y, error._position.z, angle_degr);

    //         // add new constraint to gtsam as per loop closure (TODO: needs more accurate matching between the two submaps)
    //         gtsam::Pose3 loop_measurement{ gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3{ 0, 0, 0 } };
    //         gtsam::NonlinearFactorGraph factors;
    //         using gtsam::symbol_shorthand::X;
    //         factors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(submap_i), X(submap_other_i), loop_measurement, _gtsam->_loop_noise));
    //         _gtsam->_isam.update(factors);
    //     }
    // }
    // auto Optimizer::add_submap(dag::RootIndices roots, ScanIndex scan_beg, ScanIndex scan_end) -> const Submap& {
    //     // avg of positions as submap center
    //     glm::dvec3 position{ 0, 0, 0 };
    //     for (ScanIndex scan_i = scan_beg; scan_i < scan_end; scan_i++) {
    //         const Pose& pose = _scan_poses[scan_i];
    //         position += glm::dvec3(pose._position);
    //     }
    //     position /= float(scan_end - scan_beg);

    //     // go ahead and create submap based on avg pose
    //     Submap submap{
    //         ._root_indices = roots,
    //         ._scan_beg = scan_beg,
    //         ._scan_end = scan_end,
    //         ._pose_avg{ glm::dvec3(position), glm::identity<glm::quat>() },
    //         ._pose_err{},
    //     };

    //     // add pose to gtsam
    //     using gtsam::symbol_shorthand::X;
    //     gtsam::Values values;
    //     gtsam::NonlinearFactorGraph factors;
    //     if (_submaps.size() == 0) {
    //         // anchor first submap pose
    //         gtsam::Pose3 prior_pose(gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(position.x, position.y, position.z));
    //         factors.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prior_pose, _gtsam->_prior_noise));
    //         values.insert(X(0), prior_pose);
    //         _gtsam->_isam.update(factors, values);
    //     }
    //     else {
    //         const Pose& pose_prev = _submaps.back()._pose_avg;
    //         const Pose& pose_curr = submap._pose_avg;

    //         // add new pose to pose graph
    //         gtsam::Pose3 pose{ gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(position.x, position.y, position.z) };
    //         values.insert(X(_submaps.size()), pose);

    //         // add delta to previous pose as new factor
    //         const glm::vec3 pose_delta = pose_curr._position - pose_prev._position;
    //         gtsam::Pose3 pose_delta_gtsam{ gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(pose_delta.x, pose_delta.y, pose_delta.z) };
    //         factors.add(gtsam::BetweenFactor<gtsam::Pose3>{ X(_submaps.size() - 1), X(_submaps.size()), pose_delta_gtsam, _gtsam->_odom_noise });

    //         _gtsam->_isam.update(factors, values);
    //     }

    //     _submaps.push_back(submap);
    //     return _submaps.back();
    // }
    // bool Optimizer::is_active_submap_done(const Pose& pose_new, ScanIndex submap_beg, float threshhold) {
    //     const Pose& pose_prev = _scan_poses[submap_beg];
    //     float distance = glm::distance(pose_prev._position, pose_new._position);
    //     // if our submap threshhold is crossed, finalize the active submap before inserting new points
    //     if (distance > threshhold) return true;
    //     else return false;
    // }
    // void Optimizer::debug_thingy() {
    //     gtsam::Values result = _gtsam->_isam.calculateEstimate();
    //     std::cout << "Final optimized poses:\n";
    //     for (uint32_t i = 0; i < result.size(); ++i) {
    //         auto res = result.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(i));
    //         auto rot = res.rotation().xyz();
    //         auto pos = res.translation();

    //         const Pose& pose = _submaps[i]._pose_avg;
    //         const Pose pose_true{
    //             glm::aligned_dvec3(pos.x(), pos.y(), pos.z()),
    //             glm::aligned_dvec3(rot.x(), rot.y(), rot.z())
    //         };
    //         // adjust pose error as per gtsam graph
    //         _submaps[i]._pose_err = {
    //             pose._position - pose_true._position,
    //             pose_true._rotation
    //         };
    //         fmt::println("position was ({:.2f},{:.2f},{:.2f}) and should be ({:.2f},{:.2f},{:.2f})",
    //             pose._position.x, pose._position.y, pose._position.z,
    //             pos.x(), pos.y(), pos.z()
    //         );

    //     }
    // }
}
