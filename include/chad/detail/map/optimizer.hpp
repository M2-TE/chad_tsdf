#pragma once
#include "chad/detail/ndd/ndd.hpp"
#include "chad/detail/dag/storage.hpp"
#include "chad/detail/funcs/timing.hpp"
#include "chad/detail/map/submap.hpp"
#include "chad/detail/map/indices.hpp"
#include "chad/detail/map/active_submap.hpp"

namespace chad::detail::map {
    struct Optimizer {
        Optimizer(dag::Storage& dag, float sdf_res, float sdf_trunc, float submap_xyz_threshhold, float submap_cor_threshhold);
        ~Optimizer();

        void add_scan(std::vector<glm::aligned_vec3>&& points,
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
                    _active_threads[_active_i] = std::jthread{ [this, active_submap_p]() {
                        auto timestamp = std::chrono::steady_clock::now();
                        on_submap_completion(*active_submap_p);
                        MEASURE_TIME(timestamp, "Submap completed (async)");
                    }};

                    // swap submap chain to continue work
                    _active_i = (_active_i + 1) % ACTIVE_SUBMAP_COUNT;
                    active_submap_p = &_active_submaps[_active_i];
                }
            }

            // wait for the descriptor construction to finish
            auto timestamp = std::chrono::steady_clock::now();
            descriptor_thread.join();
            MEASURE_TIME(timestamp, "Waited for descriptor");

            // Sub-Submap: check whether NDD correlation threshhold was crossed
            if (!active_submap_p->_sub_poses.empty()) {
                auto timestamp = std::chrono::steady_clock::now();
                const auto& descriptor_latest = _descriptors[active_submap_p->_descriptor_indices.back()];
                const auto [correlation, rotation] = descriptor_latest.estimate_correlation(descriptor);
                if (correlation < _submap_cor_threshhold) {
                    // let main thread handle sub-submap completion (including loop closure)
                    on_sub_submap_completion(*active_submap_p, std::move(descriptor));
                    MEASURE_TIME(timestamp, "Sub-submap completion");
                }
            }
            // Sub-Submap: when empty, initialize it
            else {
                auto timestamp = std::chrono::steady_clock::now();
                // let main thread handle sub-submap completion (including loop closure)
                on_sub_submap_completion(*active_submap_p, std::move(descriptor));
                MEASURE_TIME(timestamp, "Sub-submap initialization");
            }

            // insert new data into active submap
            timestamp = std::chrono::steady_clock::now();
            active_submap_p->add_frame(std::move(points), normals, pose, _sdf_res, _sdf_trunc);
            MEASURE_TIME(timestamp, "Sub-submap integration");
        }

    private:
        // get type dynamically, since it is templated
        using octree_t = decltype(ActiveSubmap::_tsdf_octree);
        // at which depth the leaf nodes start
        constexpr static std::size_t DEPTH_LEAVES = 21 - octree_t::get_span();

        // create standard node in DAG tree, multiple if input octree has DEPTH_SPAN > 1
        template<std::size_t DEPTH> // TODO: template spec for leaf nodes!
        auto inline create_dag_node(dag::Storage& dag, const octree_t& octree, octree_t::NodeAddr node_addr) -> dag::Addresses {
            const octree_t::Node& node = octree._nodes[node_addr];

            // keep track of newly created dag nodes (to create their parents nodes after)
            static_assert(octree_t::get_span() == 2);
            std::array<dag::ADDR_T, 8> new_nodes_tsdfs{};
            std::array<dag::ADDR_T, 8> new_nodes_weigh{};

            // node index will go from 0 to octree_t::Node::DEPTH_CHILDREN, e.g. 8 (span==1), 64 (span==2), etc
            for (std::uint32_t node_i = 0; node_i < octree_t::Node::DEPTH_CHILDREN; node_i += 8) {
                // go over all 8 potential children
                bool empty = true;
                std::array<dag::ADDR_T, 8> new_children_tsdfs{};
                std::array<dag::ADDR_T, 8> new_children_weigh{};
                for (std::uint8_t child_i = 0; child_i < 8; child_i++) {
                    // retrieve child address
                    octree_t::NodeAddr child_addr = node._children[node_i + child_i];
                    if (child_addr == 0) continue;
                    empty = false;

                    // if it exists, recursively continue at that node
                    dag::Addresses addresses = create_dag_node<DEPTH + octree_t::get_span()>(dag, octree, child_addr);
                    new_children_tsdfs[child_i] = addresses._tsdfs;
                    new_children_weigh[child_i] = addresses._weigh;
                }
                if (empty) continue;

                // this node will always be the lowest-depth one
                constexpr std::uint32_t real_depth = DEPTH + octree_t::get_span() - 1;
                dag::Addresses addresses {
                    ._tsdfs = dag.add_node(new_children_tsdfs, real_depth),
                    ._weigh = dag.add_node(new_children_weigh, real_depth),
                };
                new_nodes_tsdfs[node_i / 8] = addresses._tsdfs;
                new_nodes_weigh[node_i / 8] = addresses._weigh;

                // DEBUG: would otherwise need to create parents here (and partially clear e.g. new_nodes_tsdfs)
                static_assert(octree_t::get_span() <= 2);
            }

            // create and return highest-level DAG node addresses
            return dag::Addresses {
                ._tsdfs = dag.add_node(new_nodes_tsdfs, DEPTH),
                ._weigh = dag.add_node(new_nodes_weigh, DEPTH),
            };
        }

        // template specialization to create leaf clusters
        template<>
        auto inline create_dag_node<DEPTH_LEAVES>(dag::Storage& dag, const octree_t& octree, octree_t::NodeAddr node_addr) -> dag::Addresses {
            static_assert(octree_t::get_span() > 1); // this would otherwise needlessly complicate things even more
            static_assert(octree_t::get_span() == 2); // NOTE: would also otherwise complicate things, got more important things to work on

            // this array contains up to octree_t::Node::DEPTH_CHILDREN leaves (e.g. 64 with span==2)
            const auto& leaves = octree._nodes[node_addr]._leaves;
            std::array<dag::ADDR_T, 8> new_leaf_clusters_tsdfs{};
            std::array<dag::ADDR_T, 8> new_leaf_clusters_weigh{};

            // node index will go from 0 to octree_t::Node::DEPTH_CHILDREN, e.g. 8 (span==1), 64 (span==2), etc
            for (std::uint32_t node_i = 0; node_i < octree_t::Node::DEPTH_CHILDREN; node_i += 8) {
                // 8 leaves will form a leaf cluster
                LeafCluster lc_tsdfs, lc_weigh;
                for (std::uint32_t leaf_i = 0; leaf_i < 8; leaf_i++) {
                    const octree_t::Leaf& leaf = leaves[node_i + leaf_i];
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
                if (!lc_weigh._weigh.is_empty()) {
                    new_leaf_clusters_tsdfs[node_i / 8] = dag.add_lc(lc_tsdfs);
                    new_leaf_clusters_weigh[node_i / 8] = dag.add_lc(lc_weigh);
                }
            }
            // NOTE: creating another dag node that contains all the leaf cluster children is only necessary when span > 1
            // create and return highest-level DAG node addresses
            return dag::Addresses {
                ._tsdfs = dag.add_node(new_leaf_clusters_tsdfs, DEPTH_LEAVES),
                ._weigh = dag.add_node(new_leaf_clusters_weigh, DEPTH_LEAVES),
            };
        }

        // finish entire submap and create DAG octree (TODO)
        void on_submap_completion(ActiveSubmap& submap) {
            // lock DAG (writing)
            std::unique_lock lock_dag{ _dag._mutex, std::defer_lock };
            if (!lock_dag.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock_dag.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: on_submap_completion() waited for DAG lock release");
            }
            // lock submap (reading)
            std::unique_lock lock_sub{ submap._mutex, std::defer_lock };
            if (!lock_sub.try_lock()) {
                auto timestamp = std::chrono::steady_clock::now();
                lock_sub.lock();
                MEASURE_TIME(timestamp, "\t-> WARNING: on_submap_completion() waited for submap lock release");
            }

            // constexpr std::uint64_t depth = DEPTH_START - 1;
            // constexpr std::uint64_t highbit = std::uint64_t(1) << 63;
            // constexpr std::uint64_t mask = (highbit >> depth * 3) - 1;

            // TODO: prefault memory ranges (virtual array) for better write speeds into DAG

            // TODO: hashmap of nodes (can use DAG addresses already) with a morton code of stronger discretization to build lower levels after

            octree_t& octree = submap._tsdf_octree;
            gtl::flat_hash_map<MortonCode, dag::ADDR_T> addresses_TODO;
            for (const auto [morton_code, node_addr]: octree._roots) {
                create_dag_node<octree_t::get_start()>(_dag, octree, node_addr);
            }
            fmt::println("after");

            // std::array<std::uint8_t,               21> path;
            // std::array<octree_t::NodeAddr,         21> read_nodes;
            // std::array<std::array<dag::ADDR_T, 8>, 21> write_nodes_tsdf;
            // std::array<std::array<dag::ADDR_T, 8>, 21> write_nodes_weight;
            // path.fill(0);
            // nodes_oct.fill(0);
            // // nodes_oct[0] = Octree::ROOT;
            // nodes_tsdf.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
            // nodes_weight.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });

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
        std::size_t                                   _active_i = 0; // index for currently active submap
        // persistent data for submaps
        std::vector<Submap> _submaps;
        // persistent data for sub-submaps
        std::vector<ndd::Descriptor>            _descriptors;
        std::vector<ndd::Descriptor::LookupKey> _lookup_keys;
        // persistent data for pose graph
        std::unique_ptr<struct GTSAMData> _gtsam; // forward declared GTSAM, since those headers are gigantic

        // OLD
        [[deprecated]] std::vector<SubmapIndex> _merged_submaps;
        [[deprecated]] std::vector<Pose>        _scan_poses;
        [[deprecated]] std::vector<SubmapIndex> _scan_submap; // for easier association
    };
}
