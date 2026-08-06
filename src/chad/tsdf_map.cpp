#include "chad/tsdf_map.hpp"
#include "chad/detail/funcs/timing.hpp"
#include "chad/detail/funcs/extract.hpp"
#include "chad/detail/map/optimizer.hpp"
#include "chad/detail/misc/pose.hpp"

// TODO: use estimated normals as NDD input?

namespace chad {
    TSDFMap::TSDFMap(float sdf_res, float sdf_trunc, float submap_xyz_threshhold, float submap_cor_threshhold):
        _sdf_res(sdf_res),
        _sdf_trunc(sdf_trunc),
        _dag_p(std::make_unique<detail::dag::Storage>()),
        _map_optimizer_p(std::make_unique<detail::map::Optimizer>(*_dag_p, sdf_res, sdf_trunc, submap_xyz_threshhold, submap_cor_threshhold)) {
    }
    TSDFMap::~TSDFMap() {
    }
    void TSDFMap::clear() {
        throw std::logic_error("Function not yet implemented: chad::TSDFMap::rebuild_hashes()");
    }
    void TSDFMap::release_hashes() {
        throw std::logic_error("Function not yet implemented: chad::TSDFMap::rebuild_hashes()");
    }
    void TSDFMap::rebuild_hashes() {
        throw std::logic_error("Function not yet implemented: chad::TSDFMap::rebuild_hashes()");
    }
    // TODO: redo
    void TSDFMap::print_memory_usage() {
        // using namespace chad::detail;
        // double mem_dag_nodes = 0;
        // double mem_dag_hashes = 0;
        // [[maybe_unused]] double mem_active_octree = 0;
        // [[maybe_unused]] double mem_gtsam = 0;
        // double mem_ndd = 0;

        // // first go over everything stored in DAG (node and hash structures)
        // DAGStorage& dag = *_dag_storage_p;
        // for (const auto& level: dag._node_levels) {
        //     mem_dag_nodes += double(level._raw_data.size() * sizeof(NodeSegment));
        //     // calculated as per https://github.com/greg7mdp/parallel-hashmap?tab=readme-ov-file#memory-usage
        //     mem_dag_hashes += double(level._addr_set.size()) / double(level._addr_set.load_factor()) * double(sizeof(decltype(level._addr_set)::size_type) + 1);
        // }
        // mem_dag_nodes += double(dag._leaf_clusters._raw_data.size() * sizeof(LeafCluster));
        // mem_dag_hashes += double(dag._leaf_clusters._addr_set.size()) / double(dag._leaf_clusters._addr_set.load_factor()) * double(sizeof(decltype(dag._leaf_clusters._addr_set)::size_type) + 1);

        // // active octree should have its "reserved" memory be counted, it is preserved across submaps for performance reasons
        // const auto& active_octree = *_active_octree_p;
        // mem_active_octree += double(active_octree._nodes.size() * sizeof(Octree::Node));
        // mem_active_octree += double(active_octree._leaves.size() * sizeof(Octree::Leaf));
        // mem_active_octree += double(active_octree._node_lookup.size()) / double(active_octree._node_lookup.load_factor()) * double(sizeof(decltype(active_octree._node_lookup)::size_type) + 1);
        // // CHAD_MESSAGE(fmt::format("\tActive Octree: {:.4f}", mem_active_octree / 1024 / 1024));

        // // NDD descriptors and their lookup keys
        // const auto& optimizer = *_map_optimizer_p;
        // mem_ndd += double(optimizer._descriptors.size() * sizeof(ndd::Descriptor));
        // mem_ndd += double(optimizer._lookup_keys.size() * sizeof(ndd::Descriptor::LookupKey));

        // CHAD_MESSAGE(fmt::format("Memory footprint in MiB. Nodes: {:.4f} Hashes: {:.4f} NDDs: {:.4f}", mem_dag_nodes / 1024 / 1024, mem_dag_hashes / 1024 / 1024, mem_ndd / 1024 / 1024));
    }
    void TSDFMap::reconstruct(const std::string& foldername, uint32_t submaps_per_chunk, bool clean_first) {
        // using namespace chad::detail;
        // // need at least one inserted scan for reconstruction
        // if (_map_optimizer_p->_scan_poses.empty()) {
        //     CHAD_MESSAGE("There are no submaps to reconstruct yet");
        //     return;
        // }
        // // finalize current active submap if needed
        // if (is_submap_active()) {
        //     CHAD_MESSAGE(">> Forcefully finalizing submap for reconstruction");
        //     finalize_active_submap();
        // }

        // // make sure the folder is clean
        // if (clean_first) std::filesystem::remove_all(foldername);
        // std::filesystem::create_directory(foldername);

        // // recontruct multiple submaps as single mesh chunks
        // Octree octree_base, octree;
        // const std::vector<map::Submap>& submaps = _map_optimizer_p->_submaps;
        // for (map::SubmapIndex chunk_i = 0; chunk_i < submaps.size(); chunk_i += submaps_per_chunk) {
        //     auto beg = std::chrono::steady_clock::now();
        //     // go over all submaps within this chunk
        //     for (map::SubmapIndex submap_offset = 0; submap_offset < submaps_per_chunk && submap_offset < submaps.size(); submap_offset++) {
        //         const map::Submap& submap = submaps[chunk_i + submap_offset];
        //         octree.insert(*_dag_storage_p, submap._roots, _sdf_trunc);

        //         // invert error to get delta from octree to global coordinate frame (octree_base)
        //         glm::vec3 octree_error = -submap._pose_err._position;
        //         octree_base.merge(octree, octree_error, _sdf_res);
        //         octree.clear();
        //     }

        //     // reconstruct 3D mesh from the merged octree
        //     std::string full_file = fmt::format("{}/chunk_{}.ply", foldername, chunk_i / submaps_per_chunk);
        //     reconstruction::reconstruct(full_file, octree_base, _sdf_res);
        //     octree_base.clear();
        //     MEASURE_TIME(beg, fmt::format(">> Reconstructing submap at \"{}\"", full_file));
        // }
    }
    void TSDFMap::insert_internal(const std::uint8_t* data_p, std::size_t data_bytes, PointFlags data_flags, const std::array<double, 3>& position, const std::array<double, 3>& rotation) {
        auto beg = std::chrono::steady_clock::now();

        // convert position and rotation into glm structs for convenience
        const detail::Pose pose{ position, rotation };

        // extract points from input -> use templating for SIMD leverage (constexpr byte width)
        auto timestamp = std::chrono::steady_clock::now();
        std::vector<glm::aligned_vec3> points_xyz = detail::funcs::extract_xyz(data_p, data_bytes, data_flags);
        MEASURE_TIME(timestamp, "Preprocessing: Extracted XYZ data from input");

        // add scan to the map optimizer (will handle sub-/submapping)
        _map_optimizer_p->add_scan(std::move(points_xyz), pose);

        MEASURE_TIME(beg, "-- Total insertion time");
    }

    // TODO: prototype for point-to-tsdf
    void TSDFMap::dothingy(std::vector<glm::vec3>& points, glm::vec3& position) {
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
}
