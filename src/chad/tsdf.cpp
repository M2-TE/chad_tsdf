#include <chrono>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_aligned.hpp>
#include <Eigen/Eigen>
#include "chad/tsdf.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/normals.hpp"

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
}

namespace chad {
    TSDFMap::TSDFMap(float voxel_resolution, float truncation_distance): _voxel_resolution(voxel_resolution), _truncation_distance(truncation_distance) {
    }
    template<> void TSDFMap::insert<glm::vec3>(const std::vector<glm::vec3>& points, const glm::vec3 position) {
        auto beg = std::chrono::high_resolution_clock::now();
        fmt::println("Inserting {} points", points.size());

        // sort points by their morton code, discretized to the voxel resolution
        auto points_mc = detail::calc_mc_from_points(points, _voxel_resolution);
        auto points_sorted = detail::sort_points_by_mc(points_mc); // TODO: eval if ret is needed
        // estimate the normal of every point
        auto normals = detail::estimate_normals(points_mc, position);

        // octree shenanigans
        update_submap(points_sorted, normals, position);

        // DEBUG
        finalize_submap();

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("total    {:.2f}", dur);
    }
    template<> void TSDFMap::insert<Eigen::Vector3f>(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f position) {
        std::vector<glm::vec3> glm_points;
        glm_points.reserve(points.size());
        for (const auto& point: points) {
            glm_points.emplace_back(point.x(), point.y(), point.z());
        }
        insert(glm_points, glm::vec3{ position.x(), position.y(), position.z() });
    }
    template<> void TSDFMap::insert<std::array<float, 3>>(const std::vector<std::array<float, 3>>& points, const std::array<float, 3> position) {
        std::vector<glm::vec3> glm_points;
        glm_points.reserve(points.size());
        for (const auto& point: points) {
            glm_points.emplace_back(point[0], point[1], point[2]);
        }
        insert(glm_points, glm::vec3{ position[0], position[1], position[2] });
    }

    void TSDFMap::update_submap(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const glm::vec3 position) {
        auto beg = std::chrono::high_resolution_clock::now();
        const float voxel_reciprocal = float(1.0 / double(_voxel_resolution));
        const glm::aligned_vec3 position_aligned = position;

        std::vector<glm::aligned_vec3> traversed_voxels;
        for (size_t i = 0; i < points.size(); i++) {
            const glm::aligned_vec3 point = points[i];
            const glm::aligned_vec3 normal = normals[i];

            // get all voxels along ray within truncation distance via variant of DDA line algorithm (-> "A fast voxel traversal algorithm for ray tracing")
            // as Bresehnham's line algorithm misses some voxels
            const glm::aligned_vec3 direction = glm::normalize(point - position_aligned);
            const glm::aligned_vec3 direction_recip = 1.0f / direction;
            const glm::aligned_vec3 start = point - direction * _truncation_distance;
            const glm::aligned_vec3 final = point + direction * _truncation_distance;
            const glm::aligned_vec3 voxel_start = glm::floor(start * voxel_reciprocal);
            const glm::aligned_vec3 voxel_final = glm::floor(final * voxel_reciprocal);

            // stepN: direction of increment for each dimension
            const glm::aligned_vec3 voxel_step_direction = glm::sign(voxel_final - voxel_start);
            // tDeltaN: distance of "direction" needed to traverse an entire voxel width
            const glm::aligned_vec3 voxel_step_delta = _voxel_resolution * direction_recip;
            // tMaxN: distance of "direction" needed for each separate dimension to cross current voxel boundary
            glm::aligned_vec3 voxel_step_max = ((voxel_start + voxel_step_direction) * _voxel_resolution - start) * direction_recip;
            
            // current voxel during traversal
            glm::aligned_vec3 voxel_current = voxel_start;
            // past-the-end voxel with slight bias to account for floating point inaccuracies
            const glm::aligned_vec3 voxel_end = voxel_final + 0.9f*voxel_step_direction;

            // traverse ray within truncation distance
            traversed_voxels.push_back(voxel_current);
            while (true) {
                if (voxel_step_max.x < voxel_step_max.y) {
                    if (voxel_step_max.x < voxel_step_max.z) {
                        voxel_current.x += voxel_step_direction.x; // step in x direction
                        voxel_step_max.x += voxel_step_delta.x; // update for next voxel boundary
                        if (voxel_current.x >= voxel_end.x) break;
                    }
                    else {
                        voxel_current.z += voxel_step_direction.z; // step in z direction
                        voxel_step_max.z += voxel_step_delta.z; // update for next voxel boundary
                        if (voxel_current.z >= voxel_end.z) break;
                    }
                }
                else {
                    if (voxel_step_max.y < voxel_step_max.z) {

                        voxel_current.y += voxel_step_direction.y; // step in y direction
                        voxel_step_max.y += voxel_step_delta.y; // update for next voxel boundary
                        if (voxel_current.y >= voxel_end.y) break;
                    }
                    else {
                        voxel_current.z += voxel_step_direction.z; // step in z direction
                        voxel_step_max.z += voxel_step_delta.z; // update for next voxel boundary
                        if (voxel_current.z >= voxel_end.z) break;
                    }
                }
                traversed_voxels.push_back(voxel_current);
            }
            
            for (const glm::aligned_vec3& voxel: traversed_voxels) {
                auto& leaf = _active_submap.insert(detail::MortonCode(voxel));

                // compute signed distance
                glm::aligned_vec3 diff = voxel - point;
                float signed_distance = glm::dot(normal, diff);
                // weighted average with incremented weight
                leaf._signed_distance = leaf._signed_distance * leaf._weight + signed_distance;
                leaf._weight++;
                leaf._signed_distance = leaf._signed_distance / leaf._weight;
            }
            traversed_voxels.clear();
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("sub upd  {:.2f}", dur);
    }
    void TSDFMap::finalize_submap() {
        auto beg = std::chrono::high_resolution_clock::now();

        // trackers for the traversed path and nodes
        std::array<uint8_t, MAX_DEPTH> path;
        std::array<uint32_t, MAX_DEPTH> nodes_oct;
        std::array<std::array<uint32_t, 8>, MAX_DEPTH> nodes_dag;
        path.fill(0);
        nodes_oct.fill(0);
        nodes_dag.fill({ 0, 0, 0, 0, 0, 0, 0, 0 });
        nodes_oct[0] = _active_submap.get_root();
        const float truncation_distance_recip = 1.0f / _truncation_distance;

        int32_t depth = 0;
        while (depth >= 0) {
            uint8_t child_i = path[depth]++;
            // fmt::println("depth {:2} child {:1}", depth, child_i);

            // when all children at this depth were iterated
            if (child_i >= 8) {
                // TODO
                depth--;
            }
            // node contains node children
            else if (depth < int32_t(MAX_DEPTH - 1)) {
                // retrieve child address
                uint32_t child_addr = _active_submap.get_child(nodes_oct[depth], child_i);
                if (child_addr == 0) continue;

                // walk deeper
                depth++;
                path[depth] = 0;
                nodes_oct[depth] = child_addr;
            }
            // node contains leaf children
            else {
                // retrieve node
                const detail::Octree::Node& node = _active_submap.get_node(nodes_oct[depth]);
                
                // create leaf cluster from all 8 leaves
                LeafCluster leaf_cluster;
                for (uint8_t leaf_i = 0; leaf_i < 8; leaf_i++) {
                    uint32_t leaf_addr = node[leaf_i];
                    if (leaf_addr == 0) leaf_cluster.set_leaf_empty(leaf_i);
                    else                {
                        float signed_distance = _active_submap.get_leaf(leaf_addr)._signed_distance;
                        leaf_cluster.set_leaf_sd(leaf_i, signed_distance, truncation_distance_recip);
                    }
                }

                // TODO: lc insert

                // fmt::println("leaf cluster");
            }
        }


        // TODO
        // detail::Submap& submap = _submaps.emplace_back();

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("sub fin  {:.2f}", dur);
    }
}