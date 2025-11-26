#include "chad/tsdf_map.hpp"
#include "chad/submap.hpp"
#include "chad/detail/dag.hpp"
#include "chad/detail/lvr2.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/normals.hpp"

namespace chad {
    TSDFMap::TSDFMap(float sdf_res, float sdf_trunc, float submap_fin_delta): _sdf_res(sdf_res), _sdf_trunc(sdf_trunc), _submap_fin_delta(submap_fin_delta) {
        _dag_p = new detail::DAG();
        _active_octree_p = new detail::Octree();
    }
    TSDFMap::~TSDFMap() {
        delete _dag_p;
        delete _active_octree_p;
    }

    void TSDFMap::insert_pointcloud(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        using namespace chad::detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // turn float array into usable vector
        const glm::vec3 position_vec { position[0], position[1], position[2] };
        const Pose pose{ position, {} };

        // check if a new submap should be created
        if (!_active_submap._poses.empty()) {
            // finalize active submap once traversed far enough
            glm::vec3 first_position = _active_submap._poses[0].get_position<glm::vec3>();
            if (glm::distance(first_position, position_vec) > _submap_fin_delta) finalize_active_submap();
        }
        // update pose
        _active_submap._poses.push_back(pose);

        // sort points by their morton code, discretized to the voxel resolution
        MortonVector points_mc = calc_morton_vector(points, _sdf_res);
        std::vector<glm::vec3> points_sorted = sort_morton_vector(points_mc);
        // estimate the normal of every point
        std::vector<glm::vec3> normals = estimate_normals(points_mc, position_vec);

        // insert points into active octree as signed distances
        _active_octree_p->insert(points_sorted, normals, position_vec, _sdf_res, _sdf_trunc);

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        std::println("total    {:.2f}\n", dur);
    }
    auto TSDFMap::insert_octree(detail::Octree* octree_p) -> Submap::Roots {
        using namespace chad::detail;
        auto beg = std::chrono::high_resolution_clock::now();
        const Octree& octree = *octree_p;

        // trackers for the traversed path and nodes
        std::array<uint8_t, DAG::MAX_DEPTH> path;
        std::array<uint32_t, DAG::MAX_DEPTH> nodes_oct; // for reading
        std::array<std::array<uint32_t, 8>, DAG::MAX_DEPTH> nodes_tsdf;   // for writing
        std::array<std::array<uint32_t, 8>, DAG::MAX_DEPTH> nodes_weight; // for writing
        path.fill(0);
        nodes_oct.fill(0);
        nodes_oct[0] = octree.get_root();
        nodes_tsdf.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
        nodes_weight.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
        const float sdf_trunc_recip = 1.0f / _sdf_trunc;

        // traverse octree to build DAG
        uint32_t depth = 0;
        Submap::Roots roots;
        while (true) {
            uint8_t child_i = path[depth]++;

            // when all children at this depth were iterated
            if (child_i >= 8) {
                // create/get nodes from current node level
                uint32_t addr_tsdf   = _dag_p->add_node(depth, nodes_tsdf  [depth]);
                uint32_t addr_weight = _dag_p->add_node(depth, nodes_weight[depth]);

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
            else if (depth < DAG::MAX_DEPTH - 1) {
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
                nodes_tsdf  [depth][child_i] = _dag_p->add_lc(lc_tsdfs);
                nodes_weight[depth][child_i] = _dag_p->add_lc(lc_weigh);
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        std::println("sub fin  {:.2f}\n", dur);

        return roots;
    }
    void TSDFMap::finalize_active_submap() {
        Submap::Roots roots = insert_octree(_active_octree_p);
        _active_submap._roots = roots;
        _active_submap.update_pose();
        _submaps.push_back(_active_submap);
        // begin new submap and octree
        _active_submap.clear();
        _active_octree_p->clear();
    }

    auto TSDFMap::merge_submaps(Submap::Handle submap_handle_a, Submap::Handle submap_handle_b) -> Submap::Handle {
        auto beg = std::chrono::high_resolution_clock::now();

        // data is temporarily written to these octrees for better memory access
        Octree octree_a, octree_b;
        octree_a.insert(*_dag_p, _submaps[submap_handle_a], _sdf_trunc);
        octree_b.insert(*_dag_p, _submaps[submap_handle_b], _sdf_trunc);

        // invert error to get delta from b to a
        // assumes a is global coordinate frame
        glm::vec3 error_delta_b_to_a = -_submaps[submap_handle_b]._pose_err.get_position<glm::vec3>();
        octree_a.insert(octree_b, error_delta_b_to_a, _sdf_res);

        // create a new DAG from the merged octree
        Submap::Roots roots = insert_octree(&octree_a);
        Submap::Handle handle = _submaps.size();
        Submap& submap = _submaps.emplace_back();
        submap._roots = roots;

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        std::println("oct merge {:.2f}", dur);

        return handle;
    }
    auto TSDFMap::merge_all_submaps() -> Submap::Handle {
        using namespace chad::detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // assume first submap is the global coordinate frame
        const Submap& submap_a = _submaps.front();

        // create temporary octrees for faster memory access
        Octree octree_a, octree_b;
        octree_a.insert(*_dag_p, submap_a, _sdf_trunc);

        if (_submaps.size() == 1) return Submap::Handle(0);
        for (uint32_t i = 1; i < _submaps.size(); i++) {
            const Submap& submap_b = _submaps[i];

            // create simple octree from submap
            octree_b.insert(*_dag_p, submap_b, _sdf_trunc);

            // invert error to get delta from b to a
            // assumes a is global coordinate frame
            glm::vec3 error_delta_b_to_a = -submap_b._pose_err.get_position<glm::vec3>();
            octree_a.insert(octree_b, error_delta_b_to_a, _sdf_res);
            octree_b.clear();
        }

        // create a new DAG from the merged octree
        Submap::Roots roots = insert_octree(&octree_a);
        Submap::Handle handle = _submaps.size();
        Submap& submap = _submaps.emplace_back();
        submap._roots = roots;

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        std::println("oct merge {:.2f}", dur);
        return handle;
    }

    // TODO: merge all submaps first
    void TSDFMap::reconstruct(const std::string& filename) {
        // finalize current active submap
        if (!_active_submap._poses.empty()) {
            finalize_active_submap();
        }

        // create meshes from all submaps
        for (uint32_t i = 0; i < _submaps.size(); i++) {
            std::string str = std::format("{}_{}", i, filename);
            reconstruct(str, i);
        }
    }
    void TSDFMap::reconstruct(const std::string& filename, Submap::Handle submap_handle) {
        // reconstruct 3D mesh using LVR2
        std::println("reconstructing a submap");
        detail::reconstruct(*_dag_p, _submaps[submap_handle], _sdf_res, _sdf_trunc, filename);
    }

    // DEBUG
    void TSDFMap::featurematching() {
        // // 2D slices to compare
        // cv::Mat2d slice_a, slice_b;
        // slice_a.create(50, 50);
        // slice_b.create(50, 50);

        // // detect features
        // std::vector<cv::KeyPoint> keypoints_a, keypoints_b;
        // auto feat_detector = brisk::BriskFeatureDetector{ 34, 4, false };
        // feat_detector.detect(slice_a, keypoints_a);
        // feat_detector.detect(slice_b, keypoints_b);

        
        // // extract descriptors
        // cv::Mat2d descriptors_a, descriptors_b;
        // auto description_extractor = brisk::BriskDescriptorExtractor{ false, false, brisk::BriskDescriptorExtractor::Version::briskV2 };
        // description_extractor.compute(slice_a, keypoints_a, descriptors_b);

        // // perform matching
        // std::vector<std::vector<cv::DMatch>> matches;
        // brisk::BruteForceMatcher matcher;
        // // automatic threshhold
        // float max_distance = 55.0f * float(descriptors_a.cols) / 48.0f;
        // matcher.radiusMatch(descriptors_b, descriptors_a, matches, max_distance);
    }
}
