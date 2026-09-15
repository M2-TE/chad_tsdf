#include "chad/detail/map/optimizer.hpp"
#include "chad/detail/ndd/nanoflann/KDTreeVectorOfVectorsAdaptor.hpp"

// all the gtsam headers
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace chad::detail::map {
    using KDTree = KDTreeVectorOfVectorsAdaptor<decltype(Optimizer::_lookup_keys), float>;
    struct GTSAMData {
        // settings that cant be constant expressions for some reason
        const gtsam::SharedDiagonal _prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished());
        const gtsam::SharedDiagonal _odom_noise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.2, 0.2, 0.2).finished());
        const gtsam::SharedDiagonal _loop_noise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.02, 0.02, 0.02, 0.1, 0.1, 0.1).finished());

        // accumulated vertices and edges that are added to the factor graph when needed
        gtsam::Values values;
        gtsam::NonlinearFactorGraph factors;

        // main GTSAM factor graph
        static constexpr int _relinearize_skip = 1; // default: 10
        static constexpr float _relinearize_threshold = 0.01; // default: 0.1
        gtsam::ISAM2 _isam{ gtsam::ISAM2Params{ gtsam::ISAM2GaussNewtonParams(), _relinearize_threshold, _relinearize_skip }};
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
    auto Optimizer::posegraph_get(SubmapIndex submap_i) -> Pose {
        using gtsam::symbol_shorthand::X;
        std::lock_guard lock{ _gtsam_mutex };
        gtsam::Pose3 pose_gtsam = _gtsam->_isam.calculateEstimate<gtsam::Pose3>(X(submap_i));
        auto pos_gtsam = pose_gtsam.translation();
        auto rot_gtsam = pose_gtsam.rotation().xyz();
        return Pose {
            glm::aligned_dvec3{ pos_gtsam.x(), pos_gtsam.x(), pos_gtsam.x() },
            glm::aligned_dvec3{ rot_gtsam.x(), rot_gtsam.y(), rot_gtsam.z() }
        };
    }
    void Optimizer::posegraph_finalize() {
        std::lock_guard lock{ _gtsam_mutex };
        _gtsam->_isam.update(_gtsam->factors, _gtsam->values);
        _gtsam->factors = gtsam::NonlinearFactorGraph{};
        _gtsam->values = gtsam::Values{};
    }
    // [on_submap_completion]: add the submap as a new factor (new position + factor to previous position)
    void Optimizer::posegraph_add_factor(SubmapIndex submap_i) {
        using gtsam::symbol_shorthand::X;

        std::lock_guard lock_gtsam{ _gtsam_mutex };
        std::lock_guard lock_submaps{ _submaps_mutex };
        const Submap& submap = _submaps[submap_i];

        // need to convert pose to gtsam::Pose3
        auto pos = submap._pose_avg._position;
        auto rot = submap._pose_avg._rotation;
        gtsam::Pose3 pose_gtsam {
            gtsam::Rot3{ gtsam::Quaternion{ rot.w, rot.x, rot.y, rot.z }},
            gtsam::Point3{ pos.x, pos.y, pos.z }
        };

        // TODO: when to call this? will have a cost
        // TODO: syncrhonize via mutex?
        // _gtsam->_isam.update(factors, values);

        // add pose to gtsam
        if (submap_i == 0) {
            // anchor first submap pose
            _gtsam->factors.add(gtsam::PriorFactor<gtsam::Pose3>{ X(0), pose_gtsam, _gtsam->_prior_noise });
            _gtsam->values.insert(X(0), pose_gtsam);
        }
        else {
            // grab the pose of our previous submap and figure out the delta
            const Pose& pose_prev = _submaps[submap_i - 1]._pose_avg;
            const Pose& pose_curr = submap._pose_avg;
            const Pose pose_delta {
                pose_curr._position - pose_prev._position,
                pose_curr._rotation * glm::inverse(pose_prev._rotation),
            };

            // add new pose to pose graph
            _gtsam->values.insert(X(submap_i), pose_gtsam);

            // add delta from previous pose as new factor
            auto posd = pose_delta._position;
            auto rotd = pose_delta._rotation;
            gtsam::Pose3 pose_delta_gtsam {
                gtsam::Rot3{ gtsam::Quaternion{ rotd.w, rotd.x, rotd.y, rotd.z }},
                gtsam::Point3{ posd.x, posd.y, posd.z }
            };
            _gtsam->factors.add(gtsam::BetweenFactor<gtsam::Pose3>{ X(submap_i - 1), X(submap_i), pose_delta_gtsam, _gtsam->_odom_noise });
        }
    }
    // [on_submap_completion]: separate function to update the kdtree for descriptor matching
    void Optimizer::update_kdtree() {
        MEASURE_DEBUG(auto timestamp = std::chrono::steady_clock::now());

        // create a new kdtree (kdtree does not overwrite yet, so it does not need to be synced)
        std::unique_lock lock_lookup_keys{ _lookup_keys_mutex };
        // TODO: check out the effectiveness of final param "n_thread_build" when building this thing takes too long
        std::uint32_t lookup_keys_n = _lookup_keys.size();
        KDTree* ndd_kdtree_new_p = new KDTree{ ndd::Descriptor::N_RINGS, _lookup_keys, 10, 1 };
        lock_lookup_keys.unlock();

        // delete old kdtree
        std::lock_guard lock_kdtree{ _ndd_kdtree_mutex };
        delete static_cast<KDTree*>(_ndd_kdtree_p);
        // set new one
        _ndd_kdtree_p = ndd_kdtree_new_p;
        _ndd_kdtree_size = lookup_keys_n;

        MEASURE_DEBUG(MEASURE_TIME(timestamp, "KDTree constructed"));
    }
    // [on_submap_completion]: when requirements for loop closure are met, perform point-to-tsdf matching to obtain error estimate
    void Optimizer::perform_loop_closure(const ActiveSubmap& active_submap, SubmapIndex submap_i) {
        // only need read access
        std::unique_lock submaps_lock{ _submaps_mutex };

        // TODO: instead of filtering by max distance here, build KDTree such that only nearby NDDs are considered
        // TODO: for above, could make a widening angle in forward direction so that loop closures behind are not considered?

        // group correlations by submap they belong to
        gtl::flat_hash_map<SubmapIndex, std::vector<ndd::Correlation>> correlation_groups;
        for (const auto& sub_submap: active_submap._sub_submaps) {
            for (const auto& correlation: sub_submap._correlations) {
                // DEBUG only allow correlations that are n descriptors apart
                if (correlation.original_descriptor_i - correlation.matching_descriptor_i < 30) continue; // TODO: parameterize

                auto [submap_i, subsubmap_i] = _submap_indices[correlation.matching_descriptor_i];
                correlation_groups[submap_i].push_back(correlation);
            }
        }

        // max distance to filter out correlations that are too far away
        double max_distance = _trajectory_distance * _trajectory_threshhold;
        double max_distance_sqr = max_distance * max_distance;

        // find the group with the most correlations
        SubmapIndex max_submap_i = 0;
        std::uint32_t max_corr_count = 0;
        for (const auto& [submap_i, correlations]: correlation_groups) {
            // check if submap is too far away to be considered
            bool valid = true;
            for (const auto& correlation: correlation_groups[max_submap_i]) {
                // active_submap
                auto [original_submap_i, original_subsubmap_i] = _submap_indices[correlation.original_descriptor_i];
                const Pose original_pose = active_submap._sub_submaps[original_subsubmap_i]._pose;
                // matching submap
                auto [matching_submap_i, matching_subsubmap_i] = _submap_indices[correlation.matching_descriptor_i];
                const Pose matching_pose = _submaps[matching_submap_i]._sub_submaps[matching_subsubmap_i]._pose;

                double distance_sqr = glm::distance2(matching_pose._position, original_pose._position);
                if (distance_sqr > max_distance_sqr) valid = false;
            }
            if (!valid) continue;

            if (max_corr_count == 0 || correlations.size() > correlation_groups[max_submap_i].size()) {
                max_submap_i = submap_i;
                max_corr_count = correlations.size();
            }
        }
        if (max_corr_count == 0) return;

        // create rough initial estimate by averaging edges between corresponding nodes
        std::uint32_t mean_shift = 0;
        glm::aligned_dvec3 mean_translation = { 0, 0, 0 };
        for (const auto& correlation: correlation_groups[max_submap_i]) {
            // active_submap
            auto [original_submap_i, original_subsubmap_i] = _submap_indices[correlation.original_descriptor_i];
            const Pose original_pose = active_submap._sub_submaps[original_subsubmap_i]._pose;
            // matching submap
            auto [matching_submap_i, matching_subsubmap_i] = _submap_indices[correlation.matching_descriptor_i];
            const Pose matching_pose = _submaps[matching_submap_i]._sub_submaps[matching_subsubmap_i]._pose;
            // accumulate the translational delta
            mean_translation += matching_pose._position - original_pose._position;
            mean_shift += correlation.sector_shift;
        }
        mean_shift /= static_cast<double>(correlation_groups[max_submap_i].size());
        mean_translation /= static_cast<double>(correlation_groups[max_submap_i].size());
        float angle_degr = static_cast<double>(mean_shift) * (360.0 / static_cast<double>(ndd::Descriptor::N_SECTORS));
        auto estimated_error_pose = Pose{ mean_translation, glm::aligned_dvec3{ 0, angle_degr, 0 } };

        // grab all the stuff we need from the matched submap before releasing the lock
        Submap matched_submap = _submaps[max_submap_i];
        dag::ADDR_T root_addr = matched_submap._roots._tsdfs;
        Pose matched_pose = matched_submap._pose_avg;
        submaps_lock.unlock();

        // gotta update isam2 with the newest factors
        std::unique_lock lock_gtsam{ _gtsam_mutex };
        using gtsam::symbol_shorthand::X;
        auto timestamp = std::chrono::steady_clock::now();
        _gtsam->_isam.update(_gtsam->factors, _gtsam->values);
        _gtsam->factors = gtsam::NonlinearFactorGraph{};
        _gtsam->values = gtsam::Values{};
        MEASURE_TIME(timestamp, "ISAM2 UPDATE TIME");

        // with updated isam, estimate pose for matched submap
        timestamp = std::chrono::steady_clock::now();
        Pose matched_pose_real = posegraph_get(max_submap_i);
        lock_gtsam.unlock();
        MEASURE_TIME(timestamp, "ISAM2 ESTIMATE TIME");

        timestamp = std::chrono::steady_clock::now();
        // sample points from the DAG
        const std::vector<glm::aligned_vec3> sampled_points = _dag.sample_points_from_tsdf(root_addr, _sdf_res, _sdf_trunc);
        MEASURE_TIME(timestamp, "POINTS SAMPLING TIME");

        timestamp = std::chrono::steady_clock::now();
        std::uint8_t i = 0;
        for (; i < _point_to_tsdf_it_limit; i++) {
            Pose err = funcs::match_points_to_tsdf(active_submap._tsdf_octree, estimated_error_pose, _sdf_res, sampled_points, matched_pose, matched_pose_real);
            estimated_error_pose = estimated_error_pose + err;
            fmt::println("{}", estimated_error_pose._position);
            double pos_delta_sqr = glm::length2(err._position);
            double rot_delta_sqr = glm::length2(glm::eulerAngles(err._rotation));
            if (pos_delta_sqr < _sdf_res * _sdf_res * _pos_delta_min && rot_delta_sqr < _rot_delta_min) break;
        }
        MEASURE_TIME(timestamp, "POINTS-TO-TSDF");

        // in case the point-to-tsdf could not settle before hitting the iteration limit, we assume the loop closure was a dud
        if (i == _point_to_tsdf_it_limit - 1) return;
        fmt::println("{}", estimated_error_pose._position);

        // add new factor to graph
        timestamp = std::chrono::steady_clock::now();
        lock_gtsam.lock();
        auto pos = estimated_error_pose._position;
        auto rot = estimated_error_pose._rotation;
        gtsam::Pose3 estimated_error_pose_gtsam{
            gtsam::Rot3{ rot.w, rot.x, rot.y, rot.z },
            gtsam::Point3{ pos.x, pos.y, pos.z }
        };
        _gtsam->factors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(submap_i), X(max_submap_i), estimated_error_pose_gtsam, _gtsam->_loop_noise));
        lock_gtsam.unlock();
        MEASURE_TIME(timestamp, "LOOP CLOSURE FACTOR UPDATE");
    }

    // [on_sub_submap_completion]: match descriptor and its key to other descriptors to find potential correlations (loop closure candidates)
    auto Optimizer::get_loop_closure_candidates(const ndd::Descriptor::LookupKey& key, DescriptorIndex descriptor_i) -> std::vector<ndd::Correlation> {
        std::lock_guard lock{ _ndd_kdtree_mutex };
        const auto& descriptor = _descriptors[descriptor_i];

        // set up knn search within tree and find neighbours for the current descriptor key
        constexpr std::uint32_t max_matches_limit = 10;
        std::uint32_t max_matches = std::min<std::uint32_t>(max_matches_limit, _ndd_kdtree_size);
        auto out_dists_sqr = std::vector<float>(max_matches);
        auto candidate_indices = std::vector<DescriptorIndex>(max_matches);
        auto knnsearch_result = nanoflann::KNNResultSet<float, DescriptorIndex>{ max_matches };
        knnsearch_result.init(candidate_indices.data(), out_dists_sqr.data());
        static_cast<const KDTree*>(_ndd_kdtree_p)->index->findNeighbors(knnsearch_result, key.data(), nanoflann::SearchParameters(10, false));

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
}
