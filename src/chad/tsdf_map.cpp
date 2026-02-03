#include "chad/indices.hpp"
#include "chad/tsdf_map.hpp"
#include "chad/detail/ndd.hpp"
#include "chad/detail/ply.hpp"
#include "chad/detail/pose.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/normals.hpp"
#include "chad/detail/dag_storage.hpp"

// TODO: turn these into normal functions
#define CHAD_MESSAGE(message) fmt::println("[CHAD] {}", message)
#define MEASURE_TIME(beg, message) fmt::println("[CHAD] {}: {:.2f}ms", message, std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - beg).count())

// TODO: move these into detail headers
namespace chad::detail {
    struct Submap {
        Submap(const Pose& pose_avg,
                const Pose& pose_err, 
                const RootIndices& root_indices,
                ScanIndex scan_beg,
                ScanIndex scan_end):
            _pose_avg(pose_avg),
            _pose_err(pose_err), 
            _root_indices(root_indices), 
            _scan_beg(scan_beg),
            _scan_end(scan_end) {
        }

        Pose _pose_avg;
        Pose _pose_err;
        RootIndices _root_indices;
        ScanIndex _scan_beg; // index of first scan
        ScanIndex _scan_end; // past-the-end index of scan
    };

    struct MapOptimizer {
        MapOptimizer() = default;
        ~MapOptimizer() = default;

        // adds a new scan and creates a descriptor + lookup key for it
        void add_scan(const std::vector<glm::vec3>& points, const Pose& pose) {
            _scan_poses.push_back(pose);
            _scan_descriptors.emplace_back(points, pose._position);
            _scan_lookup_keys.push_back(_scan_descriptors.back().get_lookup_key());
        }

        // finalizes and adds a submap (TODO: validate)
        auto add_submap(RootIndices roots, ScanIndex scan_beg, ScanIndex scan_end) -> const Submap& {
            // avg of positions as submap center
            glm::dvec3 position;
            for (ScanIndex scan_i = scan_beg; scan_i < scan_end; scan_i++) {
                const Pose& pose = _scan_poses[scan_i];
                position += glm::dvec3(pose._position);
            }
            position /= float(scan_end - scan_beg);

            // just go ahead and create submap
            Submap submap{
                Pose{ glm::vec3(position), glm::quat() },
                Pose{},
                roots,
                scan_beg,
                scan_end,
            };
            _submaps.push_back(submap);
            return _submaps.back();
        }

        // check if the current active submap has crossed the position delta threshhold
        bool is_active_submap_done(ScanIndex submap_beg, float threshhold) {
            const Pose& pose_new = _scan_poses.back();
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
    };
};

namespace chad {
    TSDFMap::TSDFMap(float sdf_res, float sdf_trunc, float submap_threshhold, uint32_t submaps_per_chunk):
        _sdf_res(sdf_res),
        _sdf_trunc(sdf_trunc),
        _submap_threshhold(submap_threshhold),
        _submaps_per_chunk(submaps_per_chunk),
        _active_octree_p(new detail::Octree()),
        _dag_storage_p(new detail::DAGStorage()),
        _map_optimizer_p(new detail::MapOptimizer()) {
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

        // check if an active submap should be finalized
        if (is_submap_active()) {
            if (_map_optimizer_p->is_active_submap_done(_active_scan_beg, _submap_threshhold)) {
                finalize_active_submap();
            }
        }
        // either way, increment our scan index
        _active_scan_end++;

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
                        uint8_t weight = std::min<uint32_t>(leaf._weight, std::numeric_limits<uint8_t>::max());
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
        if (!is_submap_active()) CHAD_MESSAGE("There is no active submap yet");

        auto beg = std::chrono::high_resolution_clock::now();
        // create persistent octree with DAG nodes
        RootIndices roots = insert_octree(_active_octree_p);
        _map_optimizer_p->add_submap(roots, _active_scan_beg, _active_scan_end);

        // start new submap with a fresh octree and new pose indices
        _active_octree_p->clear();
        _active_scan_beg = _active_scan_end;
        MEASURE_TIME(beg, "++ Finalizing submap");
    }
    void TSDFMap::reconstruct(const std::string& foldername) {
        using namespace chad::detail;
        // need at least one inserted scan for reconstruction
        if (_map_optimizer_p->_scan_poses.empty()) {
            CHAD_MESSAGE("There are no submaps to reconstruct yet");
            return;
        }
        // finalize current active submap if needed
        if (is_submap_active()) {
            CHAD_MESSAGE(">> Forcefully finalizing submap for reconstruction");
            finalize_active_submap();
        }

        // make sure the folder exists
        std::filesystem::create_directory(foldername);

        // recontruct multiple submaps as single mesh chunks
        Octree octree_a, octree_b;
        for (SubmapIndex submap_i = 0; submap_i < _map_optimizer_p->_submaps.size(); submap_i += _submaps_per_chunk) {
            auto beg = std::chrono::high_resolution_clock::now();
            const Submap& submap_a = _map_optimizer_p->_submaps[submap_i];
            octree_a.insert(*_dag_storage_p, submap_a._root_indices, _sdf_trunc);

            // other_i as an offset from submap_i
            for (SubmapIndex other_i = 0; other_i < _submaps_per_chunk; other_i++) {
                const Submap& submap_b = _map_optimizer_p->_submaps[submap_i + other_i];
                octree_b.insert(*_dag_storage_p, submap_b._root_indices, _sdf_trunc);
                
                // invert error to get delta from b to a
                // assumes a is global coordinate frame
                glm::vec3 error_b_to_a = -submap_b._pose_err._position;
                octree_a.merge(octree_b, error_b_to_a, _sdf_res);
                octree_b.clear();
            }

            // reconstruct 3D mesh from the merged octree
            std::string full_file = fmt::format("{}/chunk_{}.ply", foldername, submap_i / _submaps_per_chunk);
            ply::reconstruct(full_file, octree_a, _sdf_res);
            octree_a.clear();
            MEASURE_TIME(beg, fmt::format(">> Reconstructing submap at \"{}\"", full_file));
        }
    }
}
