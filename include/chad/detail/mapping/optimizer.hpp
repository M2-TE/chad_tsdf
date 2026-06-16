#pragma once
#include "chad/detail/dag_storage.hpp"
// #include "chad/detail/dag/storage.hpp"
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/funcs/timing.hpp"
#include "chad/detail/mapping/submap.hpp"
#include "chad/detail/mapping/indices.hpp"
#include "chad/detail/mapping/active_submap.hpp"

namespace chad::detail::mapping {
    struct Optimizer {
        Optimizer(float sdf_res, float sdf_trunc, float submap_xyz_threshhold, float submap_cor_threshhold);
        ~Optimizer();

        void add_scan(DAGStorage& dag,
                      std::vector<glm::aligned_vec3>&& points,
                      const std::vector<glm::aligned_vec3>& normals,
                      Pose pose,
                      ndd::Descriptor& descriptor,
                      std::jthread& descriptor_thread) {
            // get the active submap that is currently in use (basically a swap chain)
            ActiveSubmap* active_submap_p = &_active_submaps[_active_i];
            // make sure submap is not busy (should normally never wait, hence the try_lock)
            std::unique_lock lock{ active_submap_p->_mutex, std::defer_lock };
            if (!lock.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: Optimizer waited for submap lock release");
            }
            lock.unlock(); // TODO: really think about whether this should be unlocked early!

            // Submap: check whether translational delta threshhold was crossed
            if (!active_submap_p->_all_poses.empty()) {
                Pose pose_first = active_submap_p->_all_poses.front();
                float distance = glm::distance(pose_first._position, pose._position);
                if (distance > _submap_xyz_threshhold) {
                    // let another thread handle dag writes
                    _active_threads[_active_i] = std::jthread{ [this, active_submap_p, &dag]() {
                        auto timestamp = std::chrono::steady_clock::now();
                        on_submap_completion(*active_submap_p, dag);
                        MEASURE_TIME(timestamp, "\t-> Optimizer: submap completed (async)");
                    }};

                    // swap submap chain to continue work
                    _active_i = (_active_i + 1) % ACTIVE_SUBMAP_COUNT;
                    active_submap_p = &_active_submaps[_active_i];
                }
            }

            // wait for the descriptor construction to finish
            auto timestamp = std::chrono::steady_clock::now();
            descriptor_thread.join();
            MEASURE_TIME(timestamp, "\t-> Optimizer: waited for descriptor");

            // Sub-Submap: check whether NDD correlation threshhold was crossed
            if (!active_submap_p->_sub_poses.empty()) {
                auto timestamp = std::chrono::steady_clock::now();
                const auto& descriptor_latest = _descriptors[active_submap_p->_descriptor_indices.back()];
                const auto [correlation, rotation] = descriptor_latest.estimate_correlation(descriptor);
                if (correlation < _submap_cor_threshhold) {
                    // let main thread handle sub-submap completion (including loop closure)
                    on_sub_submap_completion(*active_submap_p, std::move(descriptor));
                    MEASURE_TIME(timestamp, "\t-> Optimizer: sub-submap completion");
                }
            }
            // Sub-Submap: when empty, initialize it
            else {
                auto timestamp = std::chrono::steady_clock::now();
                // let main thread handle sub-submap completion (including loop closure)
                on_sub_submap_completion(*active_submap_p, std::move(descriptor));
                MEASURE_TIME(timestamp, "\t-> Optimizer: sub-submap initialization");
            }

            // insert new data into active submap
            timestamp = std::chrono::steady_clock::now();
            active_submap_p->add_frame(std::move(points), normals, pose, _sdf_res, _sdf_trunc);
            MEASURE_TIME(timestamp, "\t-> Optimizer: sub-submap integration");
        }

    private:
        // finish entire submap and create DAG octree (TODO)
        void on_submap_completion(ActiveSubmap& submap, DAGStorage& dag) {
            // writing data to the DAG should be done async, so we lock the mutex
            std::unique_lock lock_dag{ dag._mutex, std::defer_lock };
            if (!lock_dag.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock_dag.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: on_submap_completion() waited for DAG lock release");
            }
            // same for the submap that we try to read
            std::unique_lock lock_sub{ submap._mutex, std::defer_lock };
            if (!lock_sub.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock_sub.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: on_submap_completion() waited for submap lock release");
            }

            // TODO: finalize as DAG octree
            // TODO: gotta consider sub-submapping cases?

            // // trackers for the traversed path and nodes
            // std::array<uint8_t, DAGStorage::MAX_DEPTH> path;
            // std::array<uint32_t, DAGStorage::MAX_DEPTH> nodes_oct; // for reading
            // std::array<std::array<uint32_t, 8>, DAGStorage::MAX_DEPTH> nodes_tsdf;   // for writing
            // std::array<std::array<uint32_t, 8>, DAGStorage::MAX_DEPTH> nodes_weight; // for writing
            // path.fill(0);
            // nodes_oct.fill(0);
            // // nodes_oct[0] = Octree::ROOT;
            // nodes_tsdf.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
            // nodes_weight.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
            // const float sdf_trunc_recip = 1.0f / _sdf_trunc;

            // // traverse octree to build DAG
            // uint32_t depth = 0;
            // detail::dag::RootIndices roots;
            // while (true) {
            //     uint8_t child_i = path[depth]++;

            //     // when all children at this depth were iterated
            //     if (child_i >= 8) {
            //         // create/get nodes from current node level
            //         uint32_t addr_tsdf   = dag.add_node(depth, nodes_tsdf  [depth]);
            //         uint32_t addr_weight = dag.add_node(depth, nodes_weight[depth]);

            //         // reset node tracker for handled nodes
            //         nodes_tsdf  [depth].fill(0);
            //         nodes_weight[depth].fill(0);

            //         // check if it's the root node
            //         if (depth == 0) {
            //             roots._tsdfs   = addr_tsdf;
            //             roots._weights = addr_weight;
            //             break;
            //         }
            //         else {
            //             // continue at parent depth
            //             depth--;
            //             // created nodes are standard tree nodes
            //             uint32_t index_in_parent = path[depth] - 1;
            //             nodes_tsdf  [depth][index_in_parent] = addr_tsdf;
            //             nodes_weight[depth][index_in_parent] = addr_weight;
            //         }
            //     }
            //     // node contains node children
            //     else if (depth < DAGStorage::MAX_DEPTH - 1) {
            //         // retrieve child address
            //         uint32_t child_addr = octree.get_node(nodes_oct[depth])[child_i];
            //         if (child_addr == 0) continue;

            //         // walk deeper
            //         depth++;
            //         path[depth] = 0;
            //         nodes_oct[depth] = child_addr;
            //     }
            //     // node contains leaf children
            //     else {
            //         // retrieve address of current child node
            //         uint32_t child_addr = octree.get_node(nodes_oct[depth])[child_i];
            //         if (child_addr == 0) continue;

            //         // retrieve node
            //         const Octree::Node& node = octree.get_node(child_addr);

            //         // create leaf cluster from all 8 leaves
            //         LeafCluster lc_tsdfs, lc_weigh;
            //         for (uint8_t leaf_i = 0; leaf_i < 8; leaf_i++) {
            //             uint32_t leaf_addr = node[leaf_i];
            //             if (leaf_addr == 0) {
            //                 lc_tsdfs._tsdfs.set_empty(leaf_i);
            //                 lc_weigh._weigh.set_empty(leaf_i);
            //             }
            //             else {
            //                 const auto& leaf = octree.get_leaf(leaf_addr);
            //                 // weight can be above 255, so we cap it at the uint8_t limit
            //                 uint8_t weight = std::min<uint32_t>(leaf._weight, std::numeric_limits<uint8_t>::max());
            //                 lc_tsdfs._tsdfs.set(leaf_i, leaf._signed_distance, sdf_trunc_recip);
            //                 lc_weigh._weigh.set(leaf_i, weight);
            //             }
            //         }
            //         // add the leaf clusters and remember their addresses
            //         nodes_tsdf  [depth][child_i] = dag.add_lc(lc_tsdfs);
            //         nodes_weight[depth][child_i] = dag.add_lc(lc_weigh);
            //     }
            // }

            // clean up everything to start a new submap
            submap.clear();
        }

        // finish only the sub-submap
        void on_sub_submap_completion(ActiveSubmap& submap, ndd::Descriptor&& descriptor) {
            std::unique_lock lock{ submap._mutex, std::defer_lock };
            if (!lock.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: on_sub_submap_completion() waited for submap lock release");
            }

            // TODO: LOOP CLOSURE HERE! -> only need to update the lookup_key kd tree after every ~5 (or all above threshhold) descriptor insertions (based on how many prev ones to skip)
            // TODO: use point-to-tsdf for more accurate err estimation after loop closure
            // TODO: store the best few candidates for matches and find the best ones once ENTIRE SUBMAP is about to be finished
            // -> relying on single sub-submap to sub-submap matches would be too unreliable

            // clear out all sub-submap data
            submap.clear_sub();
            // add index to the descriptor referring to this new sub-submap
            submap._descriptor_indices.push_back(_descriptors.size());
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
        constexpr static std::size_t ACTIVE_SUBMAP_COUNT = 2; // multiple frames-in-flight for smoother parallelization
        std::array<ActiveSubmap, ACTIVE_SUBMAP_COUNT> _active_submaps;
        std::array<std::jthread, ACTIVE_SUBMAP_COUNT> _active_threads;
        std::size_t _active_i = 0; // index for currently active submap (TODO: atomic?)
        std::jthread _dag_thread;
        // persistent data per submap
        std::vector<Submap> _submaps;
        // persistent data per sub-submap
        std::vector<ndd::Descriptor>            _descriptors;
        std::vector<ndd::Descriptor::LookupKey> _lookup_keys;
        // persistent data for pose graph
        std::unique_ptr<struct GTSAMData> _gtsam;

        // OLD
        // [[deprecated]] ActiveSubmap _active_submap; // TODO: should have 2 of these for async purposes (akin to 2 frames in flight)
        [[deprecated]] std::vector<SubmapIndex> _merged_submaps;
        [[deprecated]] std::vector<Pose>        _scan_poses;
        [[deprecated]] std::vector<SubmapIndex> _scan_submap; // for easier association
    };
}
