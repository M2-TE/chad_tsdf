#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/funcs/sort.hpp"
#include "chad/detail/funcs/timing.hpp"
#include "chad/detail/funcs/normals.hpp"
#include "chad/detail/funcs/point_to_tsdf.hpp"
#include "chad/detail/dag/storage.hpp"
#include "chad/detail/map/submap.hpp"
#include "chad/detail/map/active_submap.hpp"

// debug flag for some extra measurements
#if false
#define MEASURE_DEBUG(a) a
#else
#define MEASURE_DEBUG(a)
#endif

namespace chad::detail::map {
    struct Optimizer {
        Optimizer(dag::Storage& dag, float sdf_res, float sdf_trunc, float submap_xyz_threshhold, float submap_cor_threshhold);
        ~Optimizer();

        // adds a single scan to the map, alongside a pose (should not be multiple accumulated scans, need to raycast from pose)
        void add_scan(std::vector<glm::aligned_vec3>&& points, Pose pose) {
            // async: create a scan context descriptor from the pointcloud (copy points to avoid data race)
            detail::ndd::Descriptor descriptor;
            std::jthread descriptor_thread{[&descriptor, points, pose]() {
                descriptor = detail::ndd::Descriptor{ points, pose._position };
            }};

            // async: sort points and estimate normals
            std::vector<glm::aligned_vec3> normals;
            std::jthread points_normals_thread{[this, &normals, &points, pose]() {
                detail::funcs::sort(points, _sdf_res);
                normals = detail::funcs::estimate_normals(points, pose._position, _sdf_res);
            }};

            // update trajectory by adding new pose edge
            std::unique_lock trajectory_lock{ _trajectory_mutex };
            if (_descriptors.size() > 0) {
                _trajectory_distance += glm::distance(_trajectory_last_pose, pose._position);
            }
            _trajectory_last_pose = pose._position;
            trajectory_lock.unlock();

            // make sure submap is not busy (should normally never wait, hence the try_lock)
            ActiveSubmap* active_submap_p = &_active_submaps[_active_i];
            std::unique_lock lock_active_submap{ active_submap_p->_mutex, std::defer_lock };
            if (!lock_active_submap.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock_active_submap.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: Optimizer waited for submap lock release");
            }

            // Submap: check whether translational delta threshhold was crossed
            if (!active_submap_p->_all_poses.empty()) {
                Pose pose_first = active_submap_p->_all_poses.front();
                float distance = glm::distance(pose_first._position, pose._position);
                if (distance > _submap_xyz_threshhold) {
                    // create new submap (still empty)
                    std::unique_lock lock_sub{ _submap_mutex };
                    std::size_t submap_i = _submaps.size();
                    _submaps.emplace_back(active_submap_p->_all_poses, active_submap_p->_sub_submaps);
                    lock_sub.unlock();

                    // let another thread handle dag writes
                    lock_active_submap.unlock();
                    _active_threads[_active_i] = std::jthread{ [this, active_submap_p, submap_i]() {
                        on_submap_completion(*active_submap_p, submap_i);
                    }};

                    // swap to other submap from chain to continue work
                    _active_i = (_active_i + 1) % ACTIVE_SUBMAP_COUNT;
                    active_submap_p = &_active_submaps[_active_i];

                    lock_active_submap = std::unique_lock{ active_submap_p->_mutex, std::defer_lock };
                    if (!lock_active_submap.try_lock()) {
                        auto timestamp = std::chrono::steady_clock::now();
                        lock_active_submap.lock();
                        MEASURE_TIME(timestamp, "\t-> WARNING: optimizer::add_scan() waited for active_submap lock release");
                    }
                }
            }

            // wait for the descriptor construction to finish
            MEASURE_DEBUG(auto timestamp = std::chrono::steady_clock::now());
            descriptor_thread.join();
            MEASURE_DEBUG(MEASURE_TIME(timestamp, "Waited for descriptor"));

            // Sub-Submap: check whether NDD correlation threshhold was crossed
            if (!active_submap_p->_sub_submaps.empty()) {
                auto timestamp = std::chrono::steady_clock::now();
                const auto& sub_submap_latest = active_submap_p->_sub_submaps.back();
                const auto& descriptor_latest = _descriptors[sub_submap_latest._descriptor_i];
                const auto [correlation, rotation] = descriptor_latest.estimate_correlation(descriptor);
                if (correlation < _submap_cor_threshhold) {
                    // let main thread handle sub-submap completion (including loop closure)
                    on_sub_submap_completion(*active_submap_p, pose, std::move(descriptor));
                    MEASURE_TIME(timestamp, "Sub-submap completion");
                }
            }
            else {
                // with no sub-submaps present yet, initialize it
                on_sub_submap_completion(*active_submap_p, pose, std::move(descriptor));
            }

            // wait for the point sort and normal estimation to finish
            MEASURE_DEBUG(timestamp = std::chrono::steady_clock::now());
            points_normals_thread.join();
            MEASURE_DEBUG(MEASURE_TIME(timestamp, "Waited for point sort and normal estimation"));

            // insert new data into active submap
            MEASURE_DEBUG(timestamp = std::chrono::steady_clock::now());
            active_submap_p->add_frame(std::move(points), normals, pose, _sdf_res, _sdf_trunc);
            MEASURE_DEBUG(MEASURE_TIME(timestamp, "Sub-submap integration"));
        }
        // finalize active submap if it contains any data
        void finalize() {
            map::ActiveSubmap& active_submap = _active_submaps[_active_i];
            std::unique_lock lock_active_submap{ active_submap._mutex };
            if (active_submap._all_poses.empty()) return;

            // create new submap (still empty)
            std::unique_lock lock_sub{ _submap_mutex };
            std::size_t submap_i = _submaps.size();
            _submaps.emplace_back(active_submap._all_poses, active_submap._sub_submaps);
            lock_sub.unlock();

            // let another thread handle dag writes
            lock_active_submap.unlock();
            map::ActiveSubmap* active_submap_p = &active_submap;
            _active_threads[_active_i] = std::jthread{ [this, active_submap_p, submap_i]() {
                on_submap_completion(*active_submap_p, submap_i);
            }};
        }

    private:
        // [on_submap_completion]: separate function to update the kdtree for descriptor matching
        void update_kdtree();
        // [on_submap_completion]: when requirements for loop closure are met, perform point-to-tsdf matching to obtain error estimate
        void perform_loop_closure(const ActiveSubmap& active_submap) {
            std::unique_lock submaps_lock{ _submap_mutex };

            // TODO: instead of filtering by max distance here, build KDTree such that only nearby NDDs are considered
            // TODO: for above, could make a widening angle in forward direction so that loop closures behind are not considered?

            // group correlations by submap they belong to
            gtl::flat_hash_map<SubmapIndex, std::vector<ndd::Correlation>> correlation_groups;
            for (const auto& sub_submap: active_submap._sub_submaps) {
                for (const auto& correlation: sub_submap._correlations) {
                    auto [submap_i, subsubmap_i] = _submap_indices[correlation.matching_descriptor_i];
                    correlation_groups[submap_i].push_back(correlation);
                }
            }

            // max distance to filter out correlations that are too far away
            double max_distance = _trajectory_distance * 0.9f; // TODO: parameterize the 10%
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

            // TODO: how to incorporate accumulated submap error?
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
            fmt::println("using simulated error of {}", glm::aligned_dvec3{ .015, -.034, .07 });
            auto estimated_error_pose = Pose{ mean_translation + glm::aligned_dvec3{ .015, -.034, .07 }, glm::aligned_dvec3{ 0, angle_degr, 0 } };

            Submap matched_submap = _submaps[max_submap_i];
            dag::ADDR_T root_addr = matched_submap._roots._tsdfs;
            Pose matched_pose = matched_submap._pose_avg;
            Pose matched_err = matched_submap._pose_err;
            submaps_lock.unlock();

            auto timestamp = std::chrono::steady_clock::now();
            // sample points from the DAG
            const std::vector<glm::aligned_vec3> sampled_points = _dag.sample_points_from_tsdf(root_addr, _sdf_res, _sdf_trunc);
            MEASURE_TIME(timestamp, "POINTS SAMPLING TIME");

            timestamp = std::chrono::steady_clock::now();
            // fmt::println("{}", estimated_error_pose._position);
            std::uint8_t i = 0;
            constexpr std::uint8_t iteration_limit = 10;
            for (; i < iteration_limit; i++) { // TODO: parameterize max iterations
                Pose err = funcs::match_points_to_tsdf(active_submap._tsdf_octree, estimated_error_pose, _sdf_res, sampled_points, matched_pose, matched_err);
                estimated_error_pose = estimated_error_pose + err;
                // fmt::println("{}", estimated_error_pose._position);
                double pos_delta_sqr = glm::length2(err._position);
                double rot_delta_sqr = glm::length2(glm::eulerAngles(err._rotation));
                 // TODO: parameterize these threshholds
                if (pos_delta_sqr < _sdf_res * _sdf_res * 0.2f && rot_delta_sqr < 0.1) break;
            }

            // in case the point-to-tsdf could not settle before hitting the iteration limit, we assume the loop closure was a dud
            if (i == iteration_limit - 1) return;
            fmt::println("{}", estimated_error_pose._position);
            MEASURE_TIME(timestamp, "POINTS-TO-TSDF");

            std::exit(0);



            // // add new constraint to gtsam as per loop closure (TODO: needs more accurate matching between the two submaps)
            // gtsam::Pose3 loop_measurement{ gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3{ 0, 0, 0 } };
            // gtsam::NonlinearFactorGraph factors;
            // using gtsam::symbol_shorthand::X;
            // factors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(submap_i), X(submap_other_i), loop_measurement, _gtsam->_loop_noise));
            // _gtsam->_isam.update(factors);

            // TODO: return _pose_err for new submap! (REMOVE PARAM)
        }
        // [on_submap_completion]: builds the dag tree for a single submap
        void build_dag_tree(ActiveSubmap& active_submap, SubmapIndex submap_i, std::unique_lock<std::mutex>& lock_active_sub) {
            // TODO: prefault memory ranges (virtual array) for better write speeds into DAG (should store nodes-per-level in octree?)

            // since input octree does not store lower levels, we keep track of new nodes via hashmap (2 for swapping)
            struct NodeBlueprint {
                std::array<dag::ADDR_T, 8> _tsdfs;
                std::array<dag::ADDR_T, 8> _weigh;
            };
            std::array<gtl::flat_hash_map<MortonCode, NodeBlueprint>, 2> blueprint_maps;
            std::size_t blueprint_map_i = 0;

            // get type dynamically, since it is templated
            using octree_t = decltype(ActiveSubmap::_tsdf_octree);

            // NOTE: hardcoded assumptions during dev to make things easier
            static_assert(octree_t::_DEPTH_SPAN == 2);
            static_assert(octree_t::_DEPTH_START == 17);

            // keep track of newly created dag nodes (to create their parents nodes after)
            std::array<std::array<dag::ADDR_T, 8>, 3> new_nodes_tsdfs{};
            std::array<std::array<dag::ADDR_T, 8>, 3> new_nodes_weigh{};

            // first off, convert nodes from input octree into dag nodes (up to the level where it starts)
            // this is a bit messy since octree levels span multiple depths, whereas DAG levels are 1 depth each
            octree_t& octree = active_submap._tsdf_octree;
            for (const auto& [morton_code, node_addr]: octree._roots) {
                constexpr std::uint64_t depth = octree_t::_DEPTH_START;
                const octree_t::Node& node = octree._nodes[node_addr];
                new_nodes_tsdfs[0].fill(0); // reset
                new_nodes_weigh[0].fill(0); // reset

                // iterate over 64 children
                for (std::uint32_t node_i = 0; node_i < octree_t::Node::DEPTH_CHILDREN; node_i += 8) {
                    new_nodes_tsdfs[1].fill(0); // reset
                    new_nodes_weigh[1].fill(0); // reset
                    // go over 8 of the children
                    bool empty = true;
                    for (std::uint32_t child_i = 0; child_i < 8; child_i++) {
                        new_nodes_tsdfs[2].fill(0); // reset
                        new_nodes_weigh[2].fill(0); // reset
                        // retrieve child address
                        octree_t::NodeAddr child_addr = node._children[node_i + child_i];
                        if (child_addr == 0) continue;
                        const auto& leaves = octree._nodes[child_addr]._leaves;
                        empty = false;

                        // iterate over the 64 children (leaves)
                        for (std::uint32_t lc_i = 0; lc_i < octree_t::Node::DEPTH_CHILDREN; lc_i += 8) {
                            // 8 leaves will form a leaf cluster
                            LeafCluster lc_tsdfs{};
                            LeafCluster lc_weigh{};
                            for (std::uint32_t leaf_i = 0; leaf_i < 8; leaf_i++) {
                                const octree_t::Leaf& leaf = leaves[lc_i + leaf_i];
                                if (leaf._weight == 0) {
                                    lc_tsdfs._tsdfs.set_empty(leaf_i);
                                    lc_weigh._weigh.set_empty(leaf_i);
                                }
                                else {
                                    // weight can be above 255, so we cap it at the uint8_t limit
                                    std::uint8_t weight = std::min<std::uint32_t>(leaf._weight, std::numeric_limits<std::uint8_t>::max());
                                    lc_tsdfs._tsdfs.set(leaf_i, leaf._signed_distance, _sdf_trunc_reciprocal);
                                    lc_weigh._weigh.set(leaf_i, weight);
                                }
                            }
                            if (lc_weigh._weigh.empty()) continue;
                            // create DAG node and store its address for later
                            new_nodes_tsdfs[2][lc_i / 8] = _dag.add_lc(lc_tsdfs);
                            new_nodes_weigh[2][lc_i / 8] = _dag.add_lc(lc_weigh);
                        }
                        // create DAG node and store its address for later
                        new_nodes_tsdfs[1][child_i] = _dag.add_node(new_nodes_tsdfs[2], depth + 2);
                        new_nodes_weigh[1][child_i] = _dag.add_node(new_nodes_weigh[2], depth + 2);
                    }
                    if (empty) continue;
                    // create DAG node and store its address for later
                    new_nodes_tsdfs[0][node_i / 8] = _dag.add_node(new_nodes_tsdfs[1], depth + 1);
                    new_nodes_weigh[0][node_i / 8] = _dag.add_node(new_nodes_weigh[1], depth + 1);
                }

                // write to address cache using higher discretization (to build parent node)
                auto [it, _] = blueprint_maps[blueprint_map_i].try_emplace(morton_code.mask<depth - 1>());
                // write address to correct child index within (still nonexistant) parent node
                std::uint64_t child_index = morton_code.child<depth - 1, 1>();
                it->second._tsdfs[child_index] = _dag.add_node(new_nodes_tsdfs[0], depth + 0);
                it->second._weigh[child_index] = _dag.add_node(new_nodes_weigh[0], depth + 0);
            }

            // clean up active submap to be able to continue writing to it in main thread
            active_submap.clear();
            lock_active_sub.unlock();

            // build the rest of the DAG levels
            std::uint64_t depth = octree_t::_DEPTH_START - 1;
            while (depth > 0) {
                // read from map A, while writing to map B
                auto& blueprint_map_read = blueprint_maps[blueprint_map_i];
                blueprint_map_i = (blueprint_map_i + 1) % 2;
                auto& blueprint_map_write = blueprint_maps[blueprint_map_i];

                // TODO: prefault upcoming DAG level, since we know how many nodes there will be?

                // address cache will contain up to 8 addresses per morton code entry
                for (const auto& [morton_code, blueprint]: blueprint_map_read) {
                    // discretize morton code further
                    auto [it, _] = blueprint_map_write.try_emplace(morton_code.mask(depth - 1));
                    // write address to correct child index within (still nonexistant) parent node
                    std::uint64_t child_index = morton_code.child<1>(depth - 1);
                    it->second._tsdfs[child_index] = _dag.add_node(blueprint._tsdfs, depth);
                    it->second._weigh[child_index] = _dag.add_node(blueprint._weigh, depth);
                }

                // proceed to previous depth above
                blueprint_map_read.clear();
                depth--;
            }

            // create final root node; should only be one!
            auto& blueprint_map = blueprint_maps[blueprint_map_i];
            if (blueprint_map.size() != 1) {
                throw std::logic_error("More than one DAG root for a single submap. Did not happen during my testing yet; either your map is too wide or some inserted points have corrupted positions.");
            }
            const auto& [morton_code, blueprint] = *blueprint_map.cbegin();
            std::lock_guard lock{ _submap_mutex };
            _submaps[submap_i]._roots = dag::Addresses{
                ._tsdfs = _dag.add_node(blueprint._tsdfs, depth),
                ._weigh = _dag.add_node(blueprint._weigh, depth),
            };
        }
        // finish entire submap and create DAG octree
        void on_submap_completion(ActiveSubmap& active_submap, SubmapIndex submap_i) {
            // update the kdtree on another thread (internally synchronized)
            std::jthread kdtree_thread{ [this]() {
                MEASURE_DEBUG(auto timestamp = std::chrono::steady_clock::now());
                update_kdtree();
                MEASURE_DEBUG(MEASURE_TIME(timestamp, "KDTree constructed"));
            }};

            // lock submap (reading)
            std::unique_lock lock_active_sub{ active_submap._mutex, std::defer_lock };
            if (!lock_active_sub.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock_active_sub.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: on_submap_completion() waited for submap lock release");
            }
            // lock DAG (writing)
            std::unique_lock lock_dag{ _dag._mutex, std::defer_lock };
            if (!lock_dag.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock_dag.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: on_submap_completion() waited for DAG lock release");
            }

            // DEBUG ////////////////////////////////////////////////////////////////////////////////////
            perform_loop_closure(active_submap);
            /////////////////////////////////////////////////////////////////////////////////////////////

            auto timestamp = std::chrono::steady_clock::now();
            build_dag_tree(active_submap, submap_i, lock_active_sub);

            // finally, ensure kdd tree is fully built and ready
            kdtree_thread.join();
            MEASURE_TIME(timestamp, ">> async: Submap completed");
        }

        // [on_sub_submap_completion]: match descriptor and its key to other descriptors to find potential correlations (loop closure candidates)
        auto get_loop_closure_candidates(DescriptorIndex descriptor_i) -> std::vector<ndd::Correlation>;
        // finish only the sub-submap
        void on_sub_submap_completion(ActiveSubmap& active_submap, Pose pose, ndd::Descriptor&& descriptor) {
            std::lock_guard lock{ _submap_mutex };

            // store lookup key and descriptor permanently
            DescriptorIndex descriptor_i = static_cast<DescriptorIndex>(_descriptors.size());
            _lookup_keys.push_back(descriptor.get_lookup_key());
            _descriptors.push_back(std::move(descriptor));

            // remember the future submap index for the newly stored descriptor
            _submap_indices.push_back({ _submaps.size(), active_submap._sub_submaps.size() });

            // add the finalized sub-submap
            active_submap._sub_submaps.push_back(SubSubmap{
                ._pose = pose,
                ._descriptor_i = descriptor_i,
                ._correlations = get_loop_closure_candidates(descriptor_i),
            });
        }

    public:
        // settings
        dag::Storage& _dag;
        const float _sdf_res;   // copied from TSDFMap::_sdf_res
        const float _sdf_trunc; // copied from TSDFMap::_sdf_trunc
        const float _sdf_res_reciprocal;
        const float _sdf_trunc_reciprocal;
        const float _submap_xyz_threshhold;
        const float _submap_cor_threshhold;

        // transient data during submapping
        constexpr static std::size_t ACTIVE_SUBMAP_COUNT = 2; // multiple frames-in-flight for smoother parallelization
        std::array<ActiveSubmap, ACTIVE_SUBMAP_COUNT> _active_submaps;
        std::array<std::jthread, ACTIVE_SUBMAP_COUNT> _active_threads;
        std::size_t                                   _active_i; // index for currently active submap

        // persistent data for submaps
        std::vector<Submap> _submaps;
        std::mutex          _submap_mutex; // mutex for all the containers of submaps, descriptors, keys, etc

        // persistent data for sub-submaps
        std::vector<std::pair<SubmapIndex, SubSubmapIndex>> _submap_indices; // to correlate descriptor indices to submap indices
        std::vector<ndd::Descriptor>                        _descriptors;
        std::vector<ndd::Descriptor::LookupKey>             _lookup_keys;

        // persistent data for loop closure things
        std::unique_ptr<struct GTSAMData> _gtsam; // forward declared GTSAM, since those headers are gigantic
        void*         _ndd_kdtree_p; // forward declaring the nanoflann kdtree as void*, since it would be a pain otherwise
        std::uint32_t _ndd_kdtree_size; // only updated after kdtree rebuilds
        std::mutex    _ndd_kdtree_mutex;

        // ETC (TODO: gotta decide which category these belong to)
        double             _trajectory_distance; // sum of distance of edges between all poses
        // double             _trajectory_distance_lc; // _trajectory_distance since last loop closure
        glm::aligned_dvec3 _trajectory_last_pose;
        glm::aligned_dvec3 _trajectory_error;
        std::mutex         _trajectory_mutex;
    };
}
