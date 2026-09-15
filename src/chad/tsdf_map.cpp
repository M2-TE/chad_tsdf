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
    TSDFMap::TSDFMap(double sdf_res, double sdf_trunc, double submap_xyz_threshhold, double submap_cor_threshhold):
        _sdf_res(sdf_res),
        _sdf_trunc(sdf_trunc),
        _dag_p(std::make_unique<detail::dag::Storage>()),
        _map_optimizer_p(std::make_unique<detail::map::Optimizer>(*_dag_p, sdf_res, sdf_trunc, submap_xyz_threshhold, submap_cor_threshhold)) {
    }
    TSDFMap::~TSDFMap() {
    }
    void TSDFMap::clear() {
        _dag_p->clear();
        // TODO: clear optimizer
        throw std::logic_error("Function not yet implemented: chad::TSDFMap::clear()");
    }
    void TSDFMap::release_hashes() {
        _dag_p->release_hashes();
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
    void TSDFMap::reconstruct(const std::string& foldername, bool clean_first) {
        using namespace chad::detail;
        // finalize current active submap if needed
        _map_optimizer_p->finalize();
        if (_map_optimizer_p->_submaps.empty()) {
            CHAD_MESSAGE("there is nothing to reconstruct!");
            return;
        }

        // hashmaps are no longer needed at this point
        release_hashes();

        // make sure the folder is clean
        if (clean_first) std::filesystem::remove_all(foldername);
        std::filesystem::create_directory(foldername);

        // create an octree that will be used for merging
        map::Octree2<17, 2> octree_main;
        Pose pose_main_orig = _map_optimizer_p->_submaps[0]._pose_avg;
        Pose pose_main_real = _map_optimizer_p->posegraph_get(0);
        glm::aligned_dquat main_rotation = glm::inverse(pose_main_real._rotation);
        glm::aligned_dvec3 main_translation = pose_main_orig._position - pose_main_real._position;

        // merge other octrees into the base one
        for (std::size_t submap_i = 0; submap_i < _map_optimizer_p->_submaps.size(); submap_i++) {
            auto timestamp = std::chrono::steady_clock::now();

            // grab the right submap and create an octree from its DAG tree
            const map::Submap& submap_other = _map_optimizer_p->_submaps[submap_i];
            map::Octree2<17, 2> octree_other{ *_dag_p, submap_other._roots, _sdf_trunc };
            Pose pose_other_orig = submap_other._pose_avg;
            Pose pose_other_real = _map_optimizer_p->posegraph_get(submap_i);

            // calculate the transform from coordinate frame octree_other (B) to octree_main (A)
            glm::aligned_dmat4x4 transform_b_to_a = glm::identity<glm::dmat4x4>();
            transform_b_to_a = glm::translate(transform_b_to_a, pose_other_real._position + main_translation); // translate to coordinate frame A
            transform_b_to_a = transform_b_to_a * glm::mat4_cast(pose_other_real._rotation * main_rotation); // rotate around origin
            transform_b_to_a = glm::translate(transform_b_to_a, -pose_other_orig._position); // translate to origin

            // merge octree_other into the main octree using the transform between their coordinate frames
            octree_main.merge(octree_other, transform_b_to_a, _sdf_res);

            double progress = static_cast<double>(submap_i + 1) / static_cast<double>(_map_optimizer_p->_submaps.size());
            MEASURE_TIME(timestamp, fmt::format(">> Reconstructing submap [merge] {:6.2f}%", progress * 100.0));
        }

        // create mesh and write it to disk
        auto beg = std::chrono::steady_clock::now();
        std::string filename = fmt::format("{}/chunk_{}.ply", foldername, 0);
        reconstruction::Ply ply{ filename, octree_main, _sdf_res, true };
        MEASURE_TIME(beg, fmt::format(">> Reconstructing \"{}\"", filename));
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
