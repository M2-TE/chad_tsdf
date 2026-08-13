#include "chad/tsdf_map.hpp"
#include "chad/detail/misc/pose.hpp"
#include "chad/detail/funcs/timing.hpp"
#include "chad/detail/funcs/extract.hpp"
#include "chad/detail/map/optimizer.hpp"
#include "chad/detail/reconstruction/ply.hpp"

// TODO: prefault memory pages
// TODO: use estimated normals as NDD input?
// TODO: another tree (addition to tsdf and weights) for ESDF with low res

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
        _dag_p->clear();
        // TODO: clear map optimizer
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
        using namespace chad::detail;
        // finalize current active submap if needed
        _map_optimizer_p->finalize();

        // make sure the folder is clean
        if (clean_first) std::filesystem::remove_all(foldername);
        std::filesystem::create_directory(foldername);

        // reconstruct every submap
        for (std::uint32_t i = 0; i < _map_optimizer_p->_submaps.size(); i++) {
            auto beg = std::chrono::steady_clock::now();

            // simply append index to the filename
            const map::Submap& submap = _map_optimizer_p->_submaps[i];
            std::string filename = fmt::format("{}/submap_{}.ply", foldername, i);

            // reconstruct 3D mesh from the merged octree
            reconstruction::reconstruct(filename, *_dag_p, submap._roots, _sdf_res, _sdf_trunc);

            MEASURE_TIME(beg, fmt::format(">> Reconstructing submap \"{}\"", filename));
        }
    }
    void TSDFMap::insert_internal(const std::uint8_t* data_p, std::size_t data_bytes, PointFlags data_flags, std::array<double, 3> position, std::array<double, 3> rotation) {
        auto beg = std::chrono::steady_clock::now();

        // convert position and rotation into glm structs for convenience
        detail::Pose pose{ position, rotation };

        // extract points from input -> use templating for SIMD leverage (constexpr byte width)
        std::vector<glm::aligned_vec3> points_xyz = detail::funcs::extract_xyz(data_p, data_bytes, data_flags);

        // add scan to the map optimizer (will handle sub-/submapping)
        _map_optimizer_p->add_scan(std::move(points_xyz), pose);

        MEASURE_TIME(beg, "-- Total insertion time");
    }
}
