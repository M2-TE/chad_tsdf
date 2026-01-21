#include "chad/indices.hpp"
#include "chad/tsdf_map.hpp"
#include "chad/detail/ndd.hpp"
#include "chad/detail/ply.hpp"
#include "chad/detail/lvr2.hpp"
#include "chad/detail/pose.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/normals.hpp"
#include "chad/detail/dag_storage.hpp"

#define MEASURE_TIME(beg, message) fmt::println("[CHAD] {}: {:.2f}ms", message, std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - beg).count());

namespace chad::detail {
    struct Submap {
        Submap(const Pose& pose, const Pose& pose_error, const RootIndices& root_indices, ScanIndex scan_start_i, ScanIndex scan_final_i):
            _pose(pose), _pose_error(pose_error), _root_indices(root_indices), _scan_start_i(scan_start_i), _scan_final_i(scan_final_i) {}
        Pose _pose;
        Pose _pose_error;
        RootIndices _root_indices;
        ScanIndex _scan_start_i; // start index into vectors of scan data
        ScanIndex _scan_final_i; // final index into vectors of scan data
    };
}

namespace chad::detail {
    struct MapOptimizer {
        MapOptimizer() = default;
        ~MapOptimizer() = default;

        // adds a new scan and creates a descriptor + lookup key for it
        void add_scan(const std::vector<glm::vec3>& points, const Pose& pose) {
            _scan_poses.push_back(pose);
            _scan_descriptors.emplace_back(points, pose._position);
            _scan_lookup_keys.push_back(_scan_descriptors.back().get_lookup_key());
        }

        // finalizes and adds a submap
        auto add_submap(RootIndices roots, ScanIndex scan_start_i, ScanIndex scan_final_i) -> const Submap& {
            // avg of positions as submap center
            glm::dvec3 position;
            for (ScanIndex scan_i = scan_start_i; scan_i <= scan_final_i; scan_i++) {
                position += glm::dvec3(_scan_poses[scan_i]._position);
            }
            position /= float(scan_final_i - scan_start_i + 1);

            // just go ahead and create submap
            Submap submap{
                Pose{ glm::vec3(position), glm::quat() },
                Pose{},
                roots,
                scan_start_i,
                scan_final_i,
            };
            _submaps.push_back(submap);
            return _submaps.back();
        }

        bool is_submap_done(const detail::Pose& pose, float submap_distance) {
            if (_submaps.size() > 0) {
                // finalize active submap once traversed far enough
                detail::Pose first_pose = _scan_poses[_submaps.back()._scan_start_i];
                return glm::distance(first_pose._position, pose._position) > submap_distance;
            }
            else return false;
        }

        // persistent data per submap
        std::vector<Submap>      _submaps;
        std::vector<SubmapIndex> _merged_submaps;

        // persistent data per scan
        std::vector<Pose>                       _scan_poses;
        std::vector<ndd::Descriptor>            _scan_descriptors;
        std::vector<ndd::Descriptor::LookupKey> _scan_lookup_keys;
    };
};

namespace chad {
    TSDFMap::TSDFMap(float sdf_res, float sdf_trunc, float submap_fin_delta): _sdf_res(sdf_res), _sdf_trunc(sdf_trunc), _submap_fin_delta(submap_fin_delta) {
        _dag_storage_p = new detail::DAGStorage();
        _map_optimizer_p = new detail::MapOptimizer();
        _active_octree_p = new detail::Octree();
    }
    TSDFMap::~TSDFMap() {
        delete _active_octree_p;
        delete _map_optimizer_p;
        delete _dag_storage_p;
    }

    void TSDFMap::insert_pointcloud(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        using namespace chad::detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // sort points by their morton code, discretized to the voxel resolution

        auto beg_intermediate = std::chrono::high_resolution_clock::now();
        MortonVector points_mc = calc_morton_vector(points, _sdf_res);
        std::vector<glm::vec3> points_sorted = sort_morton_vector(points_mc);
        if (_debug_outputs) MEASURE_TIME(beg_intermediate, "MortonCode calc and sort");

        // add pose and create descriptor for current scan
        beg_intermediate = std::chrono::high_resolution_clock::now();
        const Pose pose{ position, {} };
        _map_optimizer_p->add_scan(points_sorted, pose);
        if (_debug_outputs) MEASURE_TIME(beg_intermediate, "Adding scan to map optimizer");

        // check if a new submap should be created
        if (_active_scan_beg != _active_scan_end) {
            const Pose& first_pose = _map_optimizer_p->_scan_poses[_active_scan_beg];
            const Pose& final_pose = _map_optimizer_p->_scan_poses[_active_scan_end];
            float distance = glm::distance(first_pose._position, final_pose._position);
            if (distance > _submap_fin_delta) finalize_active_submap();
        }
        else _active_scan_end++; // include current scan in active submap

        // estimate the normal of every point
        beg_intermediate = std::chrono::high_resolution_clock::now();
        std::vector<glm::vec3> normals = estimate_normals(points_mc, pose._position);
        if (_debug_outputs) MEASURE_TIME(beg_intermediate, "Normal estimation");

        // insert points into active octree as signed distances
        beg_intermediate = std::chrono::high_resolution_clock::now();
        _active_octree_p->insert(points_sorted, normals, pose._position, _sdf_res, _sdf_trunc);
        if (_debug_outputs) MEASURE_TIME(beg_intermediate, "Update active octree");
        MEASURE_TIME(beg, "-- Total insertion time");
    }
    auto TSDFMap::insert_octree(detail::Octree* octree_p) -> RootIndices {
        using namespace chad::detail;
        const Octree& octree = *octree_p;

        // trackers for the traversed path and nodes
        std::array<uint8_t, DAGStorage::MAX_DEPTH> path;
        std::array<uint32_t, DAGStorage::MAX_DEPTH> nodes_oct; // for reading
        std::array<std::array<uint32_t, 8>, DAGStorage::MAX_DEPTH> nodes_tsdf;   // for writing
        std::array<std::array<uint32_t, 8>, DAGStorage::MAX_DEPTH> nodes_weight; // for writing
        path.fill(0);
        nodes_oct.fill(0);
        nodes_oct[0] = Octree::ROOT;
        nodes_tsdf.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
        nodes_weight.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
        const float sdf_trunc_recip = 1.0f / _sdf_trunc;

        // traverse octree to build DAG
        uint32_t depth = 0;
        RootIndices roots;
        while (true) {
            uint8_t child_i = path[depth]++;

            // when all children at this depth were iterated
            if (child_i >= 8) {
                // create/get nodes from current node level
                uint32_t addr_tsdf   = _dag_storage_p->add_node(depth, nodes_tsdf  [depth]);
                uint32_t addr_weight = _dag_storage_p->add_node(depth, nodes_weight[depth]);

                // reset node tracker for handled nodes
                nodes_tsdf  [depth].fill(0);
                nodes_weight[depth].fill(0);

                // check if it's the root node
                if (depth == 0) {
                    roots._tsdfs   = addr_tsdf;
                    roots._weights = addr_weight;
                    break;
                }
                else {
                    // continue at parent depth
                    depth--;
                    // created nodes are standard tree nodes
                    uint32_t index_in_parent = path[depth] - 1;
                    nodes_tsdf  [depth][index_in_parent] = addr_tsdf;
                    nodes_weight[depth][index_in_parent] = addr_weight;
                }
            }
            // node contains node children
            else if (depth < DAGStorage::MAX_DEPTH - 1) {
                // retrieve child address
                uint32_t child_addr = octree.get_node(nodes_oct[depth])[child_i];
                if (child_addr == 0) continue;

                // walk deeper
                depth++;
                path[depth] = 0;
                nodes_oct[depth] = child_addr;
            }
            // node contains leaf children
            else {
                // retrieve address of current child node
                uint32_t child_addr = octree.get_node(nodes_oct[depth])[child_i];
                if (child_addr == 0) continue;

                // retrieve node
                const Octree::Node& node = octree.get_node(child_addr);

                // create leaf cluster from all 8 leaves
                LeafCluster lc_tsdfs, lc_weigh;
                for (uint8_t leaf_i = 0; leaf_i < 8; leaf_i++) {
                    uint32_t leaf_addr = node[leaf_i];
                    if (leaf_addr == 0) {
                        lc_tsdfs._tsdfs.set_empty(leaf_i);
                        lc_weigh._weigh.set_empty(leaf_i);
                    }
                    else {
                        const auto& leaf = octree.get_leaf(leaf_addr);
                        // weight can be above 255, so we cap it at the uint8_t limit
                        uint8_t weight = std::min<uint8_t>(leaf._weight, std::numeric_limits<uint8_t>::max());
                        lc_tsdfs._tsdfs.set(leaf_i, leaf._signed_distance, sdf_trunc_recip);
                        lc_weigh._weigh.set(leaf_i, weight);
                    }
                }
                // add the leaf clusters and remember their addresses
                nodes_tsdf  [depth][child_i] = _dag_storage_p->add_lc(lc_tsdfs);
                nodes_weight[depth][child_i] = _dag_storage_p->add_lc(lc_weigh);
            }
        }

        return roots;
    }
    void TSDFMap::finalize_active_submap() {
        auto beg = std::chrono::high_resolution_clock::now();
        // create persistent octree with DAG nodes
        RootIndices roots = insert_octree(_active_octree_p);
        _map_optimizer_p->add_submap(roots, _active_scan_beg, _active_scan_end);

        // start new submap with a fresh octree and new pose indices
        _active_octree_p->clear();
        _active_scan_beg = ++_active_scan_end;
        if (_debug_outputs) MEASURE_TIME(beg, "++ Finalizing submap");
    }

    void TSDFMap::reconstruct(const std::string& filename) {
        // finalize current active submap
        if (_active_scan_beg != _active_scan_end) {
            finalize_active_submap();
        }
        else if (_map_optimizer_p->_scan_poses.size() == 0) {
            fmt::println("There are no submaps to reconstruct yet");
            return;
        }

        SubmapIndex merged_index;
        // merge all submaps
        if (_map_optimizer_p->_submaps.size() == 1) {
            merged_index = 0;
        }
        else {
            // create temporary octrees for faster memory access
            detail::Octree octree_a, octree_b;
            octree_a.insert(*_dag_storage_p, _map_optimizer_p->_submaps.front()._root_indices, _sdf_trunc);

            // merge sequentially
            for (uint32_t i = 1; i < uint32_t(_map_optimizer_p->_submaps.size()); i++) {
                const detail::Submap& submap_b = _map_optimizer_p->_submaps[i];

                // create simple octree from submap
                octree_b.insert(*_dag_storage_p, submap_b._root_indices, _sdf_trunc);

                // invert error to get delta from b to a
                // assumes a is global coordinate frame
                glm::vec3 error_delta_b_to_a = -submap_b._pose_error._position;
                octree_a.insert(octree_b, error_delta_b_to_a, _sdf_res);
                octree_b.clear();
            }

            // create a new DAG from the merged octree
            RootIndices roots = insert_octree(&octree_a);
            SubmapIndex index = _map_optimizer_p->_submaps.size();
            _map_optimizer_p->add_submap(roots, 0, 0); // placeholder poses
            _map_optimizer_p->_merged_submaps.push_back(index);
            merged_index = index;
        }

        // reconstruct the fully merged submap
        reconstruct(filename, merged_index);
    }
    void TSDFMap::reconstruct(const std::string& filename, SubmapIndex submap_index) {
        fmt::println("[CHAD] >> Reconstructing a submap");
        detail::Ply mesh{ filename };
        RootIndex tsdf_root = _map_optimizer_p->_submaps[submap_index]._root_indices._tsdfs;
        mesh.reconstruct(*_dag_storage_p, tsdf_root, _sdf_res, _sdf_trunc);
        return;

        std::vector<std::array<uint8_t, 3>> colors {
            {255, 0, 0},
            {0, 255, 0},
            {0, 0, 255},
            {255, 255, 0},
            {0, 255, 255},
            {255, 0, 255},
            {255, 255, 255},
        };
        // reconstruct 3D mesh using LVR2
        detail::reconstruct(*_dag_storage_p, _map_optimizer_p->_submaps[submap_index]._root_indices, _sdf_res, _sdf_trunc, filename, colors[submap_index % colors.size()]);
    }
}
