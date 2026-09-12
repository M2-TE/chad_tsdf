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
#include "chad/detail/ndd/nanoflann/KDTreeVectorOfVectorsAdaptor.hpp"

namespace chad::detail::map {
    using KDTree = KDTreeVectorOfVectorsAdaptor<decltype(Optimizer::_lookup_keys), float>;
    struct GTSAMData {
        static constexpr int _relinearize_skip = 1;
        static constexpr float _relinearize_threshold = 0.01;
        gtsam::ISAM2 _isam{ gtsam::ISAM2Params{ gtsam::ISAM2GaussNewtonParams(), _relinearize_threshold, _relinearize_skip }};
        const gtsam::SharedDiagonal _prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished());
        const gtsam::SharedDiagonal _odom_noise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.2, 0.2, 0.2).finished());
        const gtsam::SharedDiagonal _loop_noise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.02, 0.02, 0.02, 0.1, 0.1, 0.1).finished());
    };
    Optimizer::Optimizer(dag::Storage& dag, float sdf_res, float sdf_trunc, float submap_xyz_threshhold, float submap_cor_threshhold):
        _dag(dag),
        _sdf_res(sdf_res),
        _sdf_trunc(sdf_trunc),
        _sdf_res_reciprocal(static_cast<float>(1.0 / static_cast<double>(sdf_res))),
        _sdf_trunc_reciprocal(static_cast<float>(1.0 / static_cast<double>(sdf_trunc))),
        _submap_xyz_threshhold(submap_xyz_threshhold),
        _submap_cor_threshhold(submap_cor_threshhold),
        _active_i(0),
        _submaps(),
        _descriptors(),
        _lookup_keys(),
        _gtsam(std::make_unique<GTSAMData>()),
        _ndd_kdtree_p(new KDTree{ ndd::Descriptor::N_RINGS, _lookup_keys, 10, 1 }),
        _ndd_kdtree_size(0),
        _trajectory_distance(0.0),
        _trajectory_last_pose({ 0, 0, 0 }),
        _trajectory_error({ 0, 0, 0 }) {
    }
    Optimizer::~Optimizer() {
        // wait for all threads to finish their work before exiting
        for (auto& thread: _active_threads) {
            if (thread.joinable()) thread.join();
        }
        // couldnt make it a unique pointer since it is void*
        if (_ndd_kdtree_p != nullptr) delete static_cast<KDTree*>(_ndd_kdtree_p);
    }

    auto Optimizer::get_loop_closure_candidates(DescriptorIndex descriptor_i) -> std::vector<ndd::Correlation> {
        std::lock_guard lock{ _ndd_kdtree_mutex };
        const auto& key = _lookup_keys[descriptor_i];
        const auto& descriptor = _descriptors[descriptor_i];

        // set up knn search within tree and find neighbours for the current descriptor key
        constexpr std::uint32_t max_matches_limit = 10;
        std::uint32_t max_matches = std::min<std::uint32_t>(max_matches_limit, _ndd_kdtree_size);
        auto out_dists_sqr = std::vector<float>(max_matches);
        auto candidate_indices = std::vector<DescriptorIndex>(max_matches);
        auto knnsearch_result = nanoflann::KNNResultSet<float, DescriptorIndex>{ max_matches };
        knnsearch_result.init(candidate_indices.data(), out_dists_sqr.data());
        static_cast<const KDTree*>(_ndd_kdtree_p)->index->findNeighbors(knnsearch_result, key.data(), nanoflann::SearchParameters(10));

        // go over all candidates to find potential correlations
        std::vector<ndd::Correlation> correlations;
        for (const auto& candidate_i: candidate_indices) {
            // with sufficient correlation confidence, add the correlation as a match
            const ndd::Descriptor& candidate = _descriptors[candidate_i];
            auto [correlation, shift] = descriptor.estimate_correlation(candidate);
            if (correlation > _submap_cor_threshhold) {
                correlations.push_back(ndd::Correlation{
                    .confidence = static_cast<float>(correlation),
                    .sector_shift = shift,
                    .original_descriptor_i = descriptor_i,
                    .matching_descriptor_i = candidate_i,
                });
            }
        }
        return correlations;
    }
    void Optimizer::update_kdtree() {
        // create a new kdtree, which needs no synchronization
        KDTree* ndd_kdtree_new_p = new KDTree{ ndd::Descriptor::N_RINGS, _lookup_keys, 10, 1 };
        // TODO: check out the effectiveness of final param "n_thread_build" when building this thing takes too long

        // delete old kdtree
        std::lock_guard lock{ _ndd_kdtree_mutex };
        delete static_cast<KDTree*>(_ndd_kdtree_p);
        // set new one
        _ndd_kdtree_p = ndd_kdtree_new_p;
        _ndd_kdtree_size = _lookup_keys.size();
    }

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
