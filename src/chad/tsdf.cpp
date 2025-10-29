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
    // TODO: flimsy use of submaps currently, needs to be more distinct. Especially compared to other finalize() overload better naming perhaps?
    auto TSDFMap::finalize(detail::Octree* octree_p) -> Submap {
        using namespace chad::detail;
        Octree& octree = *octree_p;
        Submap submap;

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
                    submap.root_addr_tsdf   = addr_tsdf;
                    submap.root_addr_weight = addr_weight;
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
        
        return submap;
    }

    auto TSDFMap::merge_submaps(const Submap& submap_a, const Submap& submap_b) -> Submap {
        using namespace chad::detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // data is temporarily written to these octrees for better memory access
        Octree octree_a, octree_b;
        octree_a.insert(*_dag_p, submap_a, _sdf_trunc);
        octree_b.insert(*_dag_p, submap_a, _sdf_trunc);

        // calc delta between the two submaps (from A to B)
        glm::vec3 delta_a_to_b { // TODO: also needs rotation delta
            submap_b.position[0] - submap_a.position[0] - 0.01f, // DEBUG
            submap_b.position[1] - submap_a.position[1] - 0.01f, // DEBUG
            submap_b.position[2] - submap_a.position[2] - 0.01f, // DEBUG
        };

        // track node traversal
        std::array<const Octree::Node*, DAG::MAX_DEPTH - 1> path_nodes;
        std::array<uint8_t, DAG::MAX_DEPTH + 1> path_child_indices;
        path_nodes[0] = &octree_a.get_node(octree_a.get_root()); // start at root
        path_child_indices.fill(0);

        uint32_t depth = 0;
        while (true) {
            uint8_t child_i = path_child_indices[depth]++;

            // when all children at this depth were iterated
            if (child_i >= 8) {
                if (depth > 0) depth--;
                else break; // exit main loop
            }

            // node contains node children
            else if (depth < DAG::MAX_DEPTH) {
                const Octree::Node& node = *path_nodes[depth];
                uint32_t child_addr = node[child_i];

                // check if child address is valid
                if (child_addr > 0) {
                    depth++;
                    path_child_indices[depth] = 0; // reset child index for new depth
                    path_nodes[depth] = &octree_a.get_node(child_addr);
                }
            }

            // node contains leaf children
            else {
                const Octree::Node& node = *path_nodes[depth];
                uint32_t child_addr = node[child_i];
                if (child_addr == 0) continue;

                // reconstruct morton code from path
                uint64_t code = 0;
                for (uint64_t k = 0; k < DAG::MAX_DEPTH + 1; k++) {
                    uint64_t part = path_child_indices[k] - 1;
                    code |= part << uint64_t(60 - k*3);
                }
                MortonCode mc{ code };
                glm::ivec3 leaf_voxel = mc.decode();
                glm::vec3 leaf_position = glm::vec3(leaf_voxel) * _sdf_res;
                
                // voxel pos of leaf_a in b coordinate frame
                glm::vec3 leaf_position_coord_b = leaf_position + delta_a_to_b;
                // TODO: add error of B >AND< A to A pos, so that B voxel positions are still on grid intersections
                // TODO: rotation as well
                // convert back to voxel position
                leaf_position_coord_b *= 1.0f / _sdf_res;
                // get lowest corner
                glm::ivec3 leaf_chunk_coord_b = (glm::ivec3)glm::floor(leaf_position_coord_b);

                // get the 8 corners surrounding current voxel A in coordinate frame B
                Octree::Leaf leaves_b[2][2][2];
                for (int z = 0; z < 2; z++) {
                for (int y = 0; y < 2; y++) {
                for (int x = 0; x < 2; x++) {
                    glm::ivec3 neigh_voxel = leaf_chunk_coord_b + glm::ivec3(x, y, z);
                    MortonCode neigh_morton{ neigh_voxel };
                    auto [leaf_p, leaf_exists] = octree_b.try_find(neigh_morton);
                    // store copy of submap_b leaf for interpolation
                    if (!leaf_exists) leaves_b[x][y][z] = {};
                    else leaves_b[x][y][z] = *leaf_p;
                }}}

                // interpolation factor in all 3 dimensions. As coordinate frame B was not rotated, this is simple
                glm::vec3 interpolation_factor = glm::ivec3(leaf_position_coord_b) - leaf_chunk_coord_b;

                // TODO: need to consider empty leaves?
                // TODO: instead of std::lerps, do weighted interpolation early?
                
                // trinlinear interpolation between 8 neighour voxels of octree_b
                float tsdf_b;
                float wght_b;
                // interpolate on x
                float tsdf_X00 = std::lerp(leaves_b[0][0][0]._signed_distance, leaves_b[1][0][0]._signed_distance, interpolation_factor.x);
                float tsdf_X10 = std::lerp(leaves_b[0][1][0]._signed_distance, leaves_b[1][1][0]._signed_distance, interpolation_factor.x);
                float tsdf_X01 = std::lerp(leaves_b[0][0][1]._signed_distance, leaves_b[1][0][1]._signed_distance, interpolation_factor.x);
                float tsdf_X11 = std::lerp(leaves_b[0][1][1]._signed_distance, leaves_b[1][1][1]._signed_distance, interpolation_factor.x);
                float wght_X00 = std::lerp(float(leaves_b[0][0][0]._weight), float(leaves_b[1][0][0]._weight), interpolation_factor.x);
                float wght_X10 = std::lerp(float(leaves_b[0][1][0]._weight), float(leaves_b[1][1][0]._weight), interpolation_factor.x);
                float wght_X01 = std::lerp(float(leaves_b[0][0][1]._weight), float(leaves_b[1][0][1]._weight), interpolation_factor.x);
                float wght_X11 = std::lerp(float(leaves_b[0][1][1]._weight), float(leaves_b[1][1][1]._weight), interpolation_factor.x);
                // interpolate on y
                float tsdf_XY0 = std::lerp(tsdf_X00, tsdf_X10, interpolation_factor.y);
                float tsdf_XY1 = std::lerp(tsdf_X01, tsdf_X11, interpolation_factor.y);
                float wght_XY0 = std::lerp(wght_X00, wght_X10, interpolation_factor.y);
                float wght_XY1 = std::lerp(wght_X01, wght_X11, interpolation_factor.y);
                // interpolate on z
                float tsdf_XYZ = std::lerp(tsdf_XY0, tsdf_XY1, interpolation_factor.z);
                float wght_XYZ = std::lerp(wght_XY0, wght_XY1, interpolation_factor.z);
                tsdf_b = tsdf_XYZ;
                wght_b = wght_XYZ;

                // now perform weighted interpolation between tsdf_a and tsdf_b
                Octree::Leaf& leaf_a = octree_a.get_leaf(child_addr);
                leaf_a._signed_distance = leaf_a._signed_distance * float(leaf_a._weight) + tsdf_b * wght_b;
                leaf_a._signed_distance /= float(leaf_a._weight) + wght_b;
                // simply add the two weights together
                leaf_a._weight += uint32_t(wght_b);
            }
        }

        // create a new DAG from the merged octree
        Submap submap = finalize(&octree_a);

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("oct merge {:.2f}", dur);

        return submap;
    }
    void TSDFMap::save(const std::string& filename) {
        // finalize current active submap
        if (!_active_submap.positions.empty()) {
            finalize();
        }
        save(filename, _submaps.front());
    }
    void TSDFMap::save(const std::string& filename, Submap submap) {
        // reconstruct 3D mesh using LVR2
        fmt::println("reconstructing the first submap");
        detail::reconstruct(*_dag_p, submap, _sdf_res, _sdf_trunc, filename);
    }
}