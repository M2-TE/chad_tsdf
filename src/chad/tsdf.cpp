#include "chad/tsdf.hpp"
#include "chad/submap.hpp"
#include "chad/detail/dag.hpp"
#include "chad/detail/lvr2.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/normals.hpp"

namespace chad::detail {
    void inline print_vec(glm::vec3 vec) {
        fmt::println("{:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(const glm::aligned_vec3& vec) {
        fmt::println("{:.4f} {:.4f} {:.4f}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(const glm::ivec3& vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(const glm::aligned_ivec3& vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(const std::array<float, 3>& vec) {
        fmt::println("{:.2f} {:.2f} {:.2f}", vec[0], vec[1], vec[2]);
    }
}

namespace chad {
    TSDFMap::TSDFMap(float sdf_res, float sdf_trunc, float submap_fin_delta): _sdf_res(sdf_res), _sdf_trunc(sdf_trunc), _submap_fin_delta(submap_fin_delta) {
        _dag_p = new detail::DAG();
        _active_octree_p = new detail::Octree();
    }
    TSDFMap::~TSDFMap() {
        delete _dag_p;
        delete _active_octree_p;
    }
    void TSDFMap::insert_internal(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        using namespace chad::detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // turn float array into usable vector
        const glm::vec3 position_vec { position[0], position[1], position[2] };

        // either update submap or create a new one
        auto& positions = _active_submap.positions;
        if (positions.empty()) positions.push_back(position);
        else {
            // finalize active submap once traversed far enough
            glm::vec3 first_pos { positions[0][0], positions[0][1], positions[0][2] };
            if (glm::distance(first_pos, position_vec) > _submap_fin_delta) {
                finalize();
            }
            // update active submap either way
            positions.push_back(position);
        }

        // sort points by their morton code, discretized to the voxel resolution
        MortonVector points_mc = calc_morton_vector(points, _sdf_res);
        std::vector<glm::vec3> points_sorted = sort_morton_vector(points_mc);
        // estimate the normal of every point
        std::vector<glm::vec3> normals = estimate_normals(points_mc, position_vec);

        // insert points into octree with signed distances
        _active_octree_p->insert(points_sorted, normals, position_vec, _sdf_res, _sdf_trunc);

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("total    {:.2f}\n", dur);
    }
    auto TSDFMap::finalize() -> Submap {
        using namespace chad::detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // trackers for the traversed path and nodes
        Octree& octree = *_active_octree_p;
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
                    _active_submap.root_addr_tsdf   = addr_tsdf;
                    _active_submap.root_addr_weight = addr_weight;
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
                uint32_t child_addr = octree.get_child_addr(nodes_oct[depth], child_i);
                if (child_addr == 0) continue;

                // walk deeper
                depth++;
                path[depth] = 0;
                nodes_oct[depth] = child_addr;
            }
            // node contains leaf children
            else {
                // retrieve address of current child node
                uint32_t child_addr = octree.get_child_addr(nodes_oct[depth], child_i);
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

        // calc average position of submap
        glm::dvec3 position{ 0, 0, 0 };
        for (const auto& pos: _active_submap.positions) {
            position += glm::dvec3(pos[0], pos[1], pos[2]);
        }
        position /= double(_active_submap.positions.size());
        _active_submap.position[0] = float(position.x);
        _active_submap.position[1] = float(position.y);
        _active_submap.position[2] = float(position.z);
        
        // begin new submap
        _submaps.push_back(_active_submap);
        _active_submap.clear();
        _active_octree_p->clear();

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("sub fin  {:.2f}\n", dur);

        return _submaps.back();
    }

    // TODO: move to detail namespace
    auto inline do_thingy(const detail::DAG& dag, const Submap& submap, float sdf_trunc) -> chad::detail::Octree /*TODO: simplify return*/ {
        using namespace chad::detail; // TODO: simplify
        Octree octree;

        // read-only trackers for submap
        MortonCode path_mc{ 0 };
        std::array<uint8_t,  DAG::MAX_DEPTH> path_child;// child indices along path
        std::array<uint32_t, DAG::MAX_DEPTH> addr_tsdf; // TSDF addresses along path
        std::array<uint32_t, DAG::MAX_DEPTH> addr_wght; // weight addresses along path
        path_child.fill(0);
        addr_tsdf.fill(0);
        addr_wght.fill(0);
        addr_tsdf[0] = submap.root_addr_tsdf;
        addr_wght[0] = submap.root_addr_weight;

        // iterate both trees to build separate octrees
        uint32_t depth = 0;
        while (true) {
            uint8_t child_i = path_child[depth]++;

            // when all children at this depth were iterated
            if (child_i >= 8) {
                if (depth > 0) depth--;
                else break; // exit main loop
            }
            // node contains node children
            else if (depth < DAG::MAX_DEPTH - 1) {
                // try to find the child in current node
                uint32_t child_addr_tsdf = dag.get_child_addr(depth, addr_tsdf[depth], child_i);
                uint32_t child_addr_wght = dag.get_child_addr(depth, addr_wght[depth], child_i);

                // check if child address is valid (only need to check one)
                if (child_addr_tsdf > 0) {
                    depth++;
                    path_child[depth] = 0; // reset child index for new depth
                    addr_tsdf[depth] = child_addr_tsdf;
                    addr_wght[depth] = child_addr_wght;
                }
            }
            // node contains leaf children
            else {
                // try to get the leaf cluster, skip if it doesn't exist
                uint32_t child_addr_tsdf = dag.get_child_addr(DAG::MAX_DEPTH - 1, addr_tsdf[depth], child_i);
                uint32_t child_addr_wght = dag.get_child_addr(DAG::MAX_DEPTH - 1, addr_wght[depth], child_i);
                if (child_addr_tsdf == 0) continue; // only need to check one

                // fetch actual leaf cluster
                const LeafCluster& cluster_tsdf = dag.get_lc(child_addr_tsdf);
                const LeafCluster& cluster_wght = dag.get_lc(child_addr_wght);

                // reconstruct morton code from path
                uint64_t code = 0;
                for (uint64_t k = 0; k < 63/3 - 1; k++) {
                    uint64_t part = path_child[k] - 1;
                    code |= part << uint64_t(60 - k*3);
                }
                MortonCode mc{ code };

                // get the actual leaves
                uint32_t leaf_i = 0;
                for (int32_t z = 0; z <= 1; z++) {
                for (int32_t y = 0; y <= 1; y++) {
                for (int32_t x = 0; x <= 1; x++, leaf_i++) {
                    // signed distance and weight within leaf
                    auto [signed_distance, leaf_exists] = cluster_tsdf._tsdfs.try_get(leaf_i, sdf_trunc);
                    if (!leaf_exists) continue;
                    auto weight = cluster_wght._weigh.get(leaf_i);

                    // leaf index will set the 3 LSB
                    mc._value |= leaf_i;
                    
                    // now just add it
                    Octree::Leaf& leaf = octree.insert({ mc });
                    leaf._signed_distance = signed_distance;
                    leaf._weight = weight;
                }}}
            }
        }
        return octree;
    }

    void TSDFMap::DEBUG_merge_submaps(const Submap& submap_a, const Submap& submap_b) {
        using namespace chad::detail;

        // TODO: store "memory needed" into submaps from their original octrees?
        // data is temporarily written to these octrees for memory coherency
        Octree octree_a = do_thingy(*_dag_p, submap_a, _sdf_trunc);
        Octree octree_b = do_thingy(*_dag_p, submap_b, _sdf_trunc);

        // TODO: merge into octree_a or b (decide based on how trilinear interpolation should work)
        uint32_t root_a = octree_a.get_root();
        uint32_t root_b = octree_b.get_root();

        print_vec(submap_a.position);
        print_vec(submap_b.position);
    }
    void TSDFMap::save(const std::string& filename) {
        // finalize current active submap
        if (!_active_submap.positions.empty()) {
            finalize();
        }

        // reconstruct 3D mesh using LVR2
        fmt::println("reconstructing the first submap");
        detail::reconstruct(*_dag_p, _submaps.front(), _sdf_res, _sdf_trunc, filename);
    }
}