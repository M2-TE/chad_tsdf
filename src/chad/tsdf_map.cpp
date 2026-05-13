#include "chad/detail/ndd.hpp"
#include "chad/indices.hpp"
#include "chad/tsdf_map.hpp"
#include "chad/detail/ply.hpp"
#include "chad/detail/pose.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/normals.hpp"
#include "chad/detail/optimizer.hpp"
#include "chad/detail/dag_storage.hpp"

void inline CHAD_MESSAGE(std::string_view message) {
    fmt::println("[CHAD] {}", message);
}
void inline MEASURE_TIME(std::chrono::high_resolution_clock::time_point beg, std::string_view message) {
    double dur = std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - beg).count();
    fmt::println("[CHAD] {}: {:.2f}ms", message, dur);
}

namespace chad {
    TSDFMap::TSDFMap(float sdf_res, float sdf_trunc, float submap_threshhold):
        _sdf_res(sdf_res),
        _sdf_trunc(sdf_trunc),
        _submap_threshhold(submap_threshhold),
        _active_octree_p(new detail::Octree()),
        _dag_storage_p(new detail::DAGStorage()),
        _map_optimizer_p(new detail::MapOptimizer()) {
    }
    TSDFMap::~TSDFMap() {
        delete _map_optimizer_p;
        delete _active_octree_p;
        delete _dag_storage_p;
    }
    // void TSDFMap::clear() {
    // }
    // void TSDFMap::release_hashes() {
    // }
    // void TSDFMap::rebuild_hashes() {
    // }
    void TSDFMap::print_memory_usage() {
        using namespace chad::detail;
        double mem_dag_nodes = 0;
        double mem_dag_hashes = 0;
        [[maybe_unused]] double mem_active_octree = 0;
        [[maybe_unused]] double mem_gtsam = 0;
        double mem_ndd = 0;

        // first go over everything stored in DAG (node and hash structures)
        DAGStorage& dag = *_dag_storage_p;
        for (const auto& level: dag._node_levels) {
            mem_dag_nodes += double(level._raw_data.size() * sizeof(NodeSegment));
            // calculated as per https://github.com/greg7mdp/parallel-hashmap?tab=readme-ov-file#memory-usage
            mem_dag_hashes += double(level._addr_set.size()) / double(level._addr_set.load_factor()) * double(sizeof(decltype(level._addr_set)::size_type) + 1);
        }
        mem_dag_nodes += double(dag._leaf_clusters._raw_data.size() * sizeof(LeafCluster));
        mem_dag_hashes += double(dag._leaf_clusters._addr_set.size()) / double(dag._leaf_clusters._addr_set.load_factor()) * double(sizeof(decltype(dag._leaf_clusters._addr_set)::size_type) + 1);

        // active octree should have its "reserved" memory be counted, it is preserved across submaps for performance reasons
        const auto& active_octree = *_active_octree_p;
        mem_active_octree += double(active_octree._nodes.size() * sizeof(Octree::Node));
        mem_active_octree += double(active_octree._leaves.size() * sizeof(Octree::Leaf));
        mem_active_octree += double(active_octree._node_lookup.size()) / double(active_octree._node_lookup.load_factor()) * double(sizeof(decltype(active_octree._node_lookup)::size_type) + 1);
        // CHAD_MESSAGE(fmt::format("\tActive Octree: {:.4f}", mem_active_octree / 1024 / 1024));

        // NDD descriptors and their lookup keys
        const auto& optimizer = *_map_optimizer_p;
        mem_ndd += double(optimizer._scan_descriptors.size() * sizeof(ndd::Descriptor));
        mem_ndd += double(optimizer._scan_lookup_keys.size() * sizeof(ndd::Descriptor::LookupKey));

        CHAD_MESSAGE(fmt::format("Memory footprint in MiB. Nodes: {:.4f} Hashes: {:.4f} NDDs: {:.4f}", mem_dag_nodes / 1024 / 1024, mem_dag_hashes / 1024 / 1024, mem_ndd / 1024 / 1024));
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

        // check for loop closure using all descriptors within finalized submap
        beg = std::chrono::high_resolution_clock::now();
        _map_optimizer_p->detect_loop_closure(_map_optimizer_p->_submaps.size() - 1);
        if (_debug_outputs) MEASURE_TIME(beg, "Checking for loop closure");
    }
    void TSDFMap::reconstruct(const std::string& foldername, uint32_t submaps_per_chunk, bool clean_first) {
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

        // DEBUG GTSAM OUTPUT
        gtsam::Values result = _map_optimizer_p->_isam.calculateEstimate();
        std::cout << "Final optimized poses:\n";
        for (uint32_t i = 0; i < result.size(); ++i) {
            auto res = result.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(i));
            auto rot = res.rotation().xyz();
            auto pos = res.translation();

            const Pose& pose = _map_optimizer_p->_submaps[i]._pose_avg;
            const Pose pose_true{
                glm::dvec3(pos.x(), pos.y(), pos.z()),
                glm::dvec3(rot.x(), rot.y(), rot.z())
            };
            // adjust pose error as per gtsam graph
            _map_optimizer_p->_submaps[i]._pose_err = {
                pose._position - pose_true._position,
                pose_true._rotation
            };
            fmt::println("position was ({:.2f},{:.2f},{:.2f}) and should be ({:.2f},{:.2f},{:.2f})",
                pose._position.x, pose._position.y, pose._position.z,
                pos.x(), pos.y(), pos.z()
            );

        }

        // make sure the folder is clean
        if (clean_first) std::filesystem::remove_all(foldername);
        std::filesystem::create_directory(foldername);

        // recontruct multiple submaps as single mesh chunks
        Octree octree_base, octree;
        const std::vector<Submap>& submaps = _map_optimizer_p->_submaps;
        for (SubmapIndex chunk_i = 0; chunk_i < submaps.size(); chunk_i += submaps_per_chunk) {
            auto beg = std::chrono::high_resolution_clock::now();
            // go over all submaps within this chunk
            for (SubmapIndex submap_offset = 0; submap_offset < submaps_per_chunk && submap_offset < submaps.size(); submap_offset++) {
                const Submap& submap = submaps[chunk_i + submap_offset];
                octree.insert(*_dag_storage_p, submap._root_indices, _sdf_trunc);

                // invert error to get delta from octree to global coordinate frame (octree_base)
                glm::vec3 octree_error = -submap._pose_err._position;
                octree_base.merge(octree, octree_error, _sdf_res);
                octree.clear();
            }

            // reconstruct 3D mesh from the merged octree
            std::string full_file = fmt::format("{}/chunk_{}.ply", foldername, chunk_i / submaps_per_chunk);
            ply::reconstruct(full_file, octree_base, _sdf_res);
            octree_base.clear();
            MEASURE_TIME(beg, fmt::format(">> Reconstructing submap at \"{}\"", full_file));
        }
    }

    void TSDFMap::insert_pointcloud(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        using namespace chad::detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // check if an active submap should be finalized
        const Pose pose{ position, {} };
        if (is_submap_active() && _map_optimizer_p->is_active_submap_done(pose, _active_scan_beg, _submap_threshhold)) {
            finalize_active_submap();
        }
        // either way, increment scan index
        _active_scan_end++;

        // sort points by their morton code, discretized to the voxel resolution
        auto beg_intermediate = std::chrono::high_resolution_clock::now();
        MortonVector points_mc = calc_morton_vector(points, _sdf_res);
        std::vector<glm::vec3> points_sorted = sort_morton_vector(points_mc);
        if (_debug_outputs) MEASURE_TIME(beg_intermediate, "MortonCode calc and sort");

        // add pose and create descriptor for current scan (TODO: can do this on separate thread)
        beg_intermediate = std::chrono::high_resolution_clock::now();
        _map_optimizer_p->add_scan_descriptor(points_sorted, pose);
        if (_debug_outputs) MEASURE_TIME(beg_intermediate, "Adding scan to map optimizer");

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
    // prototype for point-to-tsdf
    void TSDFMap::dothingy(std::vector<glm::vec3>& points, glm::vec3& position) {
        using namespace chad::detail;

        // TEMPORARY
        RootIndex tsdf_root = _map_optimizer_p->_submaps.back()._root_indices._tsdfs;

        // accumulate count of valid comparisons and total error estimate
        float error = 0.0f;
        std::size_t count = 0;

        std::array<std::array<double, 6>, 6> H;
        for (auto& h: H) h.fill(0);
        std::array<double, 6> g;
        g.fill(0);

        // TODO: this will fetch lots of duplicate TSDF voxels, should be batched instead (std::set or something)
        const float voxel_reciprocal = float(1.0 / double(_sdf_res));
        for (const auto& point_raw: points) {

            // get tsdf voxel at current point
            const glm::ivec3 voxel_pos{ glm::floor(point_raw * voxel_reciprocal) };
            const auto [tsdf, exists] = _dag_storage_p->get_tsdf(tsdf_root, _sdf_trunc, MortonCode{ voxel_pos });
            if (!exists) continue;
            // fmt::println("cur {}", tsdf);


            // build gradients along each axis
            glm::vec3 gradient{ 0, 0, 0 };
            for (uint8_t axis_i = 0; axis_i < 3; axis_i++) {
                glm::ivec3 neigh_pos = voxel_pos;

                // get first neighbour
                neigh_pos[axis_i] -= 1;
                const auto [tsdf_a, exists_a] = _dag_storage_p->get_tsdf(tsdf_root, _sdf_trunc, MortonCode{ neigh_pos });
                if (!exists_a) continue;

                // get second neighbour
                neigh_pos[axis_i] += 2;
                const auto [tsdf_b, exists_b] = _dag_storage_p->get_tsdf(tsdf_root, _sdf_trunc, MortonCode{ neigh_pos });
                if (!exists_b) continue;


                if ((tsdf_a > 0) == (tsdf_b > 0)) {
                    gradient[axis_i] = (tsdf_b - tsdf_a) / 2;
                }
                // fmt::println("\t [{}]: a {:.4f} b {:.4f} gradient {:.4f}", axis_i, tsdf_a, tsdf_b, gradient[axis_i]);
            }
            // fmt::println("{} {} {}", gradient.x, gradient.y, gradient.z);

            // TODO: ignoring all previous gradient calcs
            // should just calc gradient from current point to TSDF surface estimation


            // make sure points are centered around (0, 0, 0)
            const glm::vec3 point = point_raw - position;
            // fmt::println("{} {} {}", point.x, point.y, point.z);

            // cross product point x gradient
            std::array<double, 6> jacobian;
            jacobian[0] = point[1] * gradient[2] - point[2] * gradient[1];
            jacobian[1] = point[2] * gradient[0] - point[0] * gradient[2];
            jacobian[2] = point[0] * gradient[1] - point[1] * gradient[0];
            jacobian[3] = gradient[0];
            jacobian[4] = gradient[1];
            jacobian[5] = gradient[2];

            // add multiplication result to h
            for (uint8_t row = 0; row < 6; row++) {
                for (uint8_t col = 0; col < 6; col++) {
                    // H += jacobian * jacobian.transpose()
                    H[row][col] += jacobian[row] * jacobian[col];
                }
                g[row] += jacobian[row] * tsdf;
            }

            // TODO: check if using floats with more prec dist is better?
            error += std::abs(tsdf);
            count++;
        }

        fmt::println("count: {} error: {}", count, error);

        lu_decomposition(H);
        auto xi = lu_solve(H, g);
        fmt::println("rot_x {:.4f}", xi[0]);
        fmt::println("rot_y {:.4f}", xi[1]);
        fmt::println("rot_z {:.4f}", xi[2]);
        fmt::println("lin_x {:.4f}", xi[3]);
        fmt::println("lin_y {:.4f}", xi[4]);
        fmt::println("lin_z {:.4f}", xi[5]);
        // xi_to_transform(xi, next_transform, center);
        // MatrixMul<float, 4, 4, 4>(next_transform, total_transform, temp_transform);
    }
}
