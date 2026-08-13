#include "chad/detail/map/optimizer.hpp"
#include "chad/detail/ndd/nanoflann/KDTreeVectorOfVectorsAdaptor.hpp"

// all the gtsam headers, mostly taken from their example
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace chad::detail::map {
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
        _sdf_res_reciprocal(1.0f / sdf_res),
        _sdf_trunc_reciprocal(1.0f / sdf_trunc),
        _submap_xyz_threshhold(submap_xyz_threshhold),
        _submap_cor_threshhold(submap_cor_threshhold) {
    }
    Optimizer::~Optimizer() {
        // wait for all threads to finish their work before exiting
        for (auto& thread: _active_threads) {
            if (thread.joinable()) thread.join();
        }
    }

    // pilfered from HATSDF
    void lu_decomposition(std::array<std::array<double, 6>, 6>& H) {
        for (int i = 0; i < 6 - 1; i++) {
            for (int k = i + 1; k < 6; k++) {
                H[k][i] /= H[i][i];
                for (int j = i + 1; j < 6; j++) {
                    H[k][j] -= H[k][i] * H[i][j];
                }
            }
        }
    }
    // pilfered from HATSDF
    auto lu_solve(const std::array<std::array<double, 6>, 6>& H, const std::array<double, 6>& g) -> std::array<double,6> {
        std::array<double, 6> x;
        for (int i = 0; i < 6; i++) {
            x[i] = g[i];
            for (int k = 0; k < i; k++) {
                x[i] -= H[i][k] * x[k];
            }
        }
        for (int i = 6 - 1; i >= 0; i--) {
            for (int k = i + 1; k < 6; k++) {
                x[i] -= H[i][k] * x[k];
            }
            x[i] /= H[i][i];
        }
        return x;
    }
    // TODO: prototype for point-to-tsdf
    void dothingy(std::vector<glm::vec3>& points, glm::vec3& position) {
        // using namespace chad::detail;

        // // TEMPORARY
        // dag::ADDR_T tsdf_root = _map_optimizer_p->_submaps.back()._roots._tsdfs;

        // // accumulate count of valid comparisons and total error estimate
        // float error = 0.0f;
        // std::size_t count = 0;

        // std::array<std::array<double, 6>, 6> H;
        // for (auto& h: H) h.fill(0);
        // std::array<double, 6> g;
        // g.fill(0);

        // // TODO: this will fetch lots of duplicate TSDF voxels, should be batched instead (std::set or something)
        // const float voxel_reciprocal = float(1.0 / double(_sdf_res));
        // for (const auto& point_raw: points) {

        //     // get tsdf voxel at current point
        //     const glm::ivec3 voxel_pos{ glm::floor(point_raw * voxel_reciprocal) };
        //     const auto [tsdf, exists] = _dag_storage_p->get_tsdf(tsdf_root, _sdf_trunc, MortonCode{ voxel_pos });
        //     if (!exists) continue;
        //     // fmt::println("cur {}", tsdf);


        //     // build gradients along each axis
        //     glm::vec3 gradient{ 0, 0, 0 };
        //     for (uint8_t axis_i = 0; axis_i < 3; axis_i++) {
        //         glm::ivec3 neigh_pos = voxel_pos;

        //         // get first neighbour
        //         neigh_pos[axis_i] -= 1;
        //         const auto [tsdf_a, exists_a] = _dag_storage_p->get_tsdf(tsdf_root, _sdf_trunc, MortonCode{ neigh_pos });
        //         if (!exists_a) continue;

        //         // get second neighbour
        //         neigh_pos[axis_i] += 2;
        //         const auto [tsdf_b, exists_b] = _dag_storage_p->get_tsdf(tsdf_root, _sdf_trunc, MortonCode{ neigh_pos });
        //         if (!exists_b) continue;


        //         if ((tsdf_a > 0) == (tsdf_b > 0)) {
        //             gradient[axis_i] = (tsdf_b - tsdf_a) / 2;
        //         }
        //         // fmt::println("\t [{}]: a {:.4f} b {:.4f} gradient {:.4f}", axis_i, tsdf_a, tsdf_b, gradient[axis_i]);
        //     }
        //     // fmt::println("{} {} {}", gradient.x, gradient.y, gradient.z);

        //     // TODO: ignoring all previous gradient calcs
        //     // should just calc gradient from current point to TSDF surface estimation


        //     // make sure points are centered around (0, 0, 0)
        //     const glm::vec3 point = point_raw - position;
        //     // fmt::println("{} {} {}", point.x, point.y, point.z);

        //     // cross product point x gradient
        //     std::array<double, 6> jacobian;
        //     jacobian[0] = point[1] * gradient[2] - point[2] * gradient[1];
        //     jacobian[1] = point[2] * gradient[0] - point[0] * gradient[2];
        //     jacobian[2] = point[0] * gradient[1] - point[1] * gradient[0];
        //     jacobian[3] = gradient[0];
        //     jacobian[4] = gradient[1];
        //     jacobian[5] = gradient[2];

        //     // add multiplication result to h
        //     for (uint8_t row = 0; row < 6; row++) {
        //         for (uint8_t col = 0; col < 6; col++) {
        //             // H += jacobian * jacobian.transpose()
        //             H[row][col] += jacobian[row] * jacobian[col];
        //         }
        //         g[row] += jacobian[row] * tsdf;
        //     }

        //     // TODO: check if using floats with more prec dist is better?
        //     error += std::abs(tsdf);
        //     count++;
        // }

        // fmt::println("count: {} error: {}", count, error);

        // funcs::lu_decomposition(H);
        // auto xi = funcs::lu_solve(H, g);
        // fmt::println("rot_x {:.4f}", xi[0]);
        // fmt::println("rot_y {:.4f}", xi[1]);
        // fmt::println("rot_z {:.4f}", xi[2]);
        // fmt::println("lin_x {:.4f}", xi[3]);
        // fmt::println("lin_y {:.4f}", xi[4]);
        // fmt::println("lin_z {:.4f}", xi[5]);
        // // xi_to_transform(xi, next_transform, center);
        // // MatrixMul<float, 4, 4, 4>(next_transform, total_transform, temp_transform);
    }

    auto Optimizer::get_loop_closure_candidates() const -> std::vector<DescriptorIndex> {
        // indices for descriptors are within submap
        Submap& submap = _submaps[submap_i];
        const uint32_t descriptor_beg = submap._scan_beg;
        const uint32_t descriptor_end = submap._scan_end;

        return {}; // TODO
    }

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
