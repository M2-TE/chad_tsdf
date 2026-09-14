#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/funcs/sort.hpp"
#include "chad/detail/funcs/timing.hpp"
#include "chad/detail/funcs/normals.hpp"
#include "chad/detail/funcs/point_to_tsdf.hpp"
#include "chad/detail/dag/storage.hpp"
#include "chad/detail/map/submap.hpp"
#include "chad/detail/map/active_submap.hpp"

// debug flag for some additional measurements
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
            if (_descriptors.size() > 0) _trajectory_distance += glm::distance(_trajectory_last_pose, pose._position);
            _trajectory_last_pose = pose._position;
            trajectory_lock.unlock();

            // make sure submap is not busy (should normally never wait, hence the try_lock)
            ActiveSubmap* active_submap_p = &_active_submaps[_active_i];
            std::unique_lock lock_active_submap{ active_submap_p->_mutex, std::defer_lock };
            delayed_lock(lock_active_submap, "Optimizer waited for active_submap lock release");

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
                    delayed_lock(lock_active_submap, "optimizer::add_scan() waited for active_submap lock release");
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
            active_submap_p->add_frame(points, normals, pose, _sdf_res, _sdf_trunc);
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

            // wait for all threads to finish their work
            for (auto& thread: _active_threads) {
                if (thread.joinable()) thread.join();
            }
        }

    private:
        // output warning msg when locking is not immediate
        void inline delayed_lock(std::unique_lock<std::mutex>& lock, std::string_view msg) {
            if (!lock.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock.lock();
                MEASURE_TIME(timestamp, fmt::format("\t-> WARNING: {}", msg));
            }
        }

        // [on_submap_completion]: TESTING
        void gtsam_add_factor(SubmapIndex submap_i);
        // [on_submap_completion]: separate function to update the kdtree, doesnt need to be immediate
        void update_kdtree();
        // [on_submap_completion]: when requirements for loop closure are met, perform point-to-tsdf matching to obtain error estimate
        void perform_loop_closure(const ActiveSubmap& active_submap, SubmapIndex submap_i);
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
            // lock_active_sub.unlock();

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
            std::jthread kdtree_thread{ [this]() { update_kdtree(); }};

            // lock submap (reading)
            std::unique_lock lock_active_sub{ active_submap._mutex, std::defer_lock };
            delayed_lock(lock_active_sub, "on_submap_completion() waited for submap lock release");
            // lock DAG (writing)
            std::unique_lock lock_dag{ _dag._mutex, std::defer_lock };
            delayed_lock(lock_dag, "on_submap_completion() waited for DAG lock release");

            auto timestamp = std::chrono::steady_clock::now();

            // add factor to graph (no updates just yet, those are expensive)
            gtsam_add_factor(submap_i);
            // attempt to find loop closure
            perform_loop_closure(active_submap, submap_i);
            // construct the full DAG tree for this submap
            build_dag_tree(active_submap, submap_i, lock_active_sub);
            lock_active_sub.unlock();

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
        const float _trajectory_threshhold = 0.1f;
        const float _pos_delta_min = 0.2f; // multiplied by _sdf_res, this value is considered squared
        const float _rot_delta_min = 0.1f; // considered squared
        const std::uint32_t _point_to_tsdf_it_limit = 10;

        // transient data during submapping
        constexpr static std::size_t ACTIVE_SUBMAP_COUNT = 2; // multiple frames-in-flight for smoother parallelization
        std::array<ActiveSubmap, ACTIVE_SUBMAP_COUNT> _active_submaps;
        std::array<std::jthread, ACTIVE_SUBMAP_COUNT> _active_threads;
        std::size_t                                   _active_i; // index for currently active submap

        // persistent data for submaps
        std::vector<Submap> _submaps;
        std::mutex          _submap_mutex; // (TODO: better naming) mutex for all the containers of submaps, descriptors, keys, etc

        // persistent data for sub-submaps
        std::vector<std::pair<SubmapIndex, SubSubmapIndex>> _submap_indices; // to correlate descriptor indices to submap indices
        std::vector<ndd::Descriptor>                        _descriptors;
        std::vector<ndd::Descriptor::LookupKey>             _lookup_keys;

        // persistent data for loop closure things
        std::unique_ptr<struct GTSAMData> _gtsam; // forward declared GTSAM, since those headers are gigantic
        std::mutex _gtsam_mutex;

        void*         _ndd_kdtree_p; // forward declaring the nanoflann kdtree as void*, since it would be a pain otherwise
        std::uint32_t _ndd_kdtree_size; // only updated after kdtree rebuilds
        std::mutex    _ndd_kdtree_mutex;

        // ETC (TODO: gotta decide which category these belong to)
        double             _trajectory_distance; // sum of all edge lengths
        glm::aligned_dvec3 _trajectory_last_pose;
        glm::aligned_dvec3 _trajectory_error;
        std::mutex         _trajectory_mutex;
        // double             _trajectory_distance_lc; // _trajectory_distance since last loop closure
    };
}
