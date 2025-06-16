#include <chrono>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_aligned.hpp>
#include <Eigen/Eigen>
#include "chad/tsdf.hpp"
#include "chad/detail/lvr2.hpp"
#include "chad/detail/levels.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/submap.hpp"
#include "chad/detail/normals.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

namespace chad::detail {
    void print_vec(glm::vec3 vec) {
        fmt::println("{:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z);
    }
    void print_vec(glm::aligned_vec3 vec) {
        fmt::println("{:.4f} {:.4f} {:.4f}", vec.x, vec.y, vec.z);
    }
    void print_vec(glm::ivec3 vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
    void print_vec(glm::aligned_ivec3 vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }

    // TODO: move these to proper headers
    void update_octree(Octree& octree, const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const glm::vec3 position, float voxel_resolution, float truncation_distance) {
        auto beg = std::chrono::high_resolution_clock::now();
        const float voxel_reciprocal = float(1.0 / double(voxel_resolution));
        const glm::aligned_vec3 position_aligned = position;

        std::vector<glm::ivec3> traversed_voxels;
        for (size_t i = 0; i < points.size(); i++) {
            const glm::aligned_vec3 point = points[i];
            const glm::aligned_vec3 normal = normals[i];

            // get all voxels along ray within truncation distance via variant of DDA line algorithm (-> "A fast voxel traversal algorithm for ray tracing")
            // as Bresehnham's line algorithm misses some voxels
            const glm::aligned_vec3 direction = glm::normalize(point - position_aligned);
            const glm::aligned_vec3 direction_recip = 1.0f / direction;
            const glm::aligned_vec3 start = point - direction * truncation_distance;
            const glm::aligned_vec3 final = point + direction * truncation_distance;
            const glm::aligned_ivec3 voxel_start = glm::aligned_ivec3(glm::floor(start * voxel_reciprocal));
            const glm::aligned_ivec3 voxel_final = glm::aligned_ivec3(glm::floor(final * voxel_reciprocal));

            // stepN: direction of increment for each dimension
            const glm::aligned_ivec3 voxel_step_direction = glm::sign(voxel_final - voxel_start);
            // tDeltaN: portion of "direction" needed to traverse full voxel
            const glm::aligned_vec3 voxel_step_delta = glm::abs(voxel_resolution * direction_recip);
            // tMaxN: portion of "direction" needed to traverse current voxel
            glm::aligned_vec3 voxel_step_max;
            // for x
            if      (voxel_step_direction.x < 0) voxel_step_max.x = voxel_resolution * std::floor(start.x * voxel_reciprocal);
            else if (voxel_step_direction.x > 0) voxel_step_max.x = voxel_resolution * std::ceil (start.x * voxel_reciprocal);
            else /*voxel_step_direction.x == 0*/ voxel_step_max.x = std::numeric_limits<float>::max();
            // for y
            if      (voxel_step_direction.y < 0) voxel_step_max.y = voxel_resolution * std::floor(start.y * voxel_reciprocal);
            else if (voxel_step_direction.y > 0) voxel_step_max.y = voxel_resolution * std::ceil (start.y * voxel_reciprocal);
            else /*voxel_step_direction.x == 0*/ voxel_step_max.y = std::numeric_limits<float>::max();
            // for z
            if      (voxel_step_direction.z < 0) voxel_step_max.z = voxel_resolution * std::floor(start.z * voxel_reciprocal);
            else if (voxel_step_direction.z > 0) voxel_step_max.z = voxel_resolution * std::ceil (start.z * voxel_reciprocal);
            else /*voxel_step_direction.x == 0*/ voxel_step_max.z = std::numeric_limits<float>::max();
            voxel_step_max = voxel_step_max - start; // distance to voxel boundaries
            voxel_step_max = glm::abs(voxel_step_max * direction_recip); // portion of "direction" needed to cross voxel boundaries
            
            // current voxel during traversal
            glm::ivec3 voxel_current = voxel_start;

            // traverse ray within truncation distance
            traversed_voxels.push_back(voxel_current);
            while (true) {
                if (voxel_step_max.x < voxel_step_max.y) {
                    if (voxel_step_max.x < voxel_step_max.z) {
                        voxel_current.x += voxel_step_direction.x; // step in x direction
                        voxel_step_max.x += voxel_step_delta.x; // update for next voxel boundary
                        if (voxel_current.x == voxel_final.x + voxel_step_direction.x) break;
                    }
                    else {
                        voxel_current.z += voxel_step_direction.z; // step in z direction
                        voxel_step_max.z += voxel_step_delta.z; // update for next voxel boundary
                        if (voxel_current.z == voxel_final.z + voxel_step_direction.z) break;
                    }
                }
                else {
                    if (voxel_step_max.y < voxel_step_max.z) {

                        voxel_current.y += voxel_step_direction.y; // step in y direction
                        voxel_step_max.y += voxel_step_delta.y; // update for next voxel boundary
                        if (voxel_current.y == voxel_final.y + voxel_step_direction.y) break;
                    }
                    else {
                        voxel_current.z += voxel_step_direction.z; // step in z direction
                        voxel_step_max.z += voxel_step_delta.z; // update for next voxel boundary
                        if (voxel_current.z == voxel_final.z + voxel_step_direction.z) break;
                    }
                }
                traversed_voxels.push_back(voxel_current);
            }
            
            for (const glm::ivec3& voxel: traversed_voxels) {
                MortonCode mc { voxel };
                auto& leaf = octree.insert(mc);

                // compute signed distance
                glm::aligned_vec3 point_to_voxel = glm::aligned_vec3(voxel) * voxel_resolution - point;
                float signed_distance = glm::dot(normal, point_to_voxel);
                // weighted average with incremented weight
                leaf._signed_distance = leaf._signed_distance * leaf._weight + signed_distance;
                leaf._weight++;
                leaf._signed_distance = leaf._signed_distance / leaf._weight;
            }
            traversed_voxels.clear();
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("sub  upd {:.2f}", dur);
    }
    void finalize_submap(const Octree& octree, Submap& submap, detail::NodeLevels& node_levels, float truncation_distance) {
        using namespace detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // trackers for the traversed path and nodes
        std::array<uint8_t,  NodeLevels::MAX_DEPTH> path;
        std::array<uint32_t, NodeLevels::MAX_DEPTH> nodes_oct;
        std::array<std::array<uint32_t, 8>, NodeLevels::MAX_DEPTH> nodes_tsdf;
        std::array<std::array<uint32_t, 8>, NodeLevels::MAX_DEPTH> nodes_weight;
        path.fill(0);
        nodes_oct.fill(0);
        nodes_oct[0] = octree.get_root();
        nodes_tsdf.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
        nodes_weight.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
        const float truncation_distance_recip = 1.0f / truncation_distance;

        uint32_t depth = 0;
        while (true) {
            uint8_t child_i = path[depth]++;

            // when all children at this depth were iterated
            if (child_i >= 8) {
                // create/get nodes from current node level
                auto& node_level = node_levels._nodes[depth];
                uint32_t addr_tsdf   = node_level.add(nodes_tsdf  [depth]);
                uint32_t addr_weight = node_level.add(nodes_weight[depth]);

                // reset node tracker for handled nodes
                nodes_tsdf  [depth].fill(0);
                nodes_weight[depth].fill(0);

                // check if it's the root node
                if (depth == 0) {
                    submap.root_addr_tsdf = addr_tsdf;
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


                    // fmt::println("depth {}: index in parent: {} addr: {}", depth + 1, index_in_parent, addr_tsdf);
                    // static int COUNTER = 0;
                    // if (COUNTER++ >= 20) exit(0);
                }
            }
            // node contains node children
            else if (depth < NodeLevels::MAX_DEPTH - 1) {
                // retrieve child address
                uint32_t child_addr = octree.get_child_addr(nodes_oct[depth], child_i);
                if (child_addr == 0) continue;

                // walk deeper
                depth++;
                path[depth] = 0;
                nodes_oct[depth] = child_addr;
            }
            // node contains leaf cluster children
            else {
                // retrieve child address
                uint32_t child_addr = octree.get_child_addr(nodes_oct[depth], child_i);
                if (child_addr == 0) continue;

                // retrieve node
                const Octree::Node& node = octree.get_node(child_addr);
                
                // create leaf cluster from all 8 leaves
                LeafCluster lc_tsdf, lc_weight;
                for (uint8_t leaf_i = 0; leaf_i < 8; leaf_i++) {
                    uint32_t leaf_addr = node[leaf_i];
                    if (leaf_addr == 0) lc_tsdf.set_leaf_sd_empty(leaf_i);
                    else {
                        const auto& leaf = octree.get_leaf(leaf_addr);
                        lc_tsdf.set_leaf_sd(leaf_i, leaf._signed_distance, truncation_distance_recip);
                        lc_weight.set_leaf_weight(leaf_i, leaf._weight);
                    }
                }
                nodes_tsdf  [depth][child_i] = node_levels._leaf_clusters.add(lc_tsdf);
                nodes_weight[depth][child_i] = node_levels._leaf_clusters.add(lc_weight);
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("sub fin  {:.2f}", dur);
    }
}

namespace chad {
    TSDFMap::TSDFMap(float voxel_resolution, float truncation_distance): _voxel_resolution(voxel_resolution), _truncation_distance(truncation_distance) {
        _active_octree_p = new detail::Octree();
        _active_submap_p = new detail::Submap();
        _node_levels_p = new detail::NodeLevels();
    }
    TSDFMap::~TSDFMap() {
        delete _node_levels_p;
        delete _active_octree_p;
        for (detail::Submap* submap_p: _submaps) {
            delete submap_p;
        }
    }
    void TSDFMap::insert(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        using namespace detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // turn float array into usable vector
        const glm::vec3 position_vec { position[0], position[1], position[2] };

        // either update submap or create a new one
        auto& positions = _active_submap_p->positions;
        if (positions.empty()) positions.push_back(position_vec);
        else {
            // finalize active submap once traversed far enough
            glm::vec3 start = positions.front();
            float dist_sqr = 5.0f * 5.0f;
            if (glm::distance2(position_vec, start) > dist_sqr) {
                finalize_submap(*_active_octree_p, *_active_submap_p, *_node_levels_p, _truncation_distance);
                _submaps.push_back(_active_submap_p);
                _active_submap_p = new Submap();
                _active_submap_p->positions.push_back(position_vec);
                _active_octree_p->clear();
            }
            // else just update active submap
            else positions.push_back(position_vec);
        }

        // sort points by their morton code, discretized to the voxel resolution
        MortonVector points_mc = calc_morton_vector(points, _voxel_resolution);
        std::vector<glm::vec3> points_sorted = sort_morton_vector(points_mc);
        // estimate the normal of every point
        std::vector<glm::vec3> normals = estimate_normals(points_mc, position_vec);

        // octree shenanigans
        update_octree(*_active_octree_p, points_sorted, normals, position_vec, _voxel_resolution, _truncation_distance);

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("total    {:.2f}", dur);
    }
    void TSDFMap::save(const std::string& filename) {
        // finalize current active submap
        if (!_active_submap_p->positions.empty()) {
            finalize_submap(*_active_octree_p, *_active_submap_p, *_node_levels_p, _truncation_distance);
            _submaps.push_back(_active_submap_p);
        }

        // reconstruct 3D mesh using LVR2
        fmt::println("reconstructing the first submap");
        detail::reconstruct(*_submaps.front(), *_node_levels_p, _voxel_resolution, _truncation_distance, filename);
    }
}