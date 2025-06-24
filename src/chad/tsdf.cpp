#include <chrono>
#include <fmt/base.h>
#include "chad/tsdf.hpp"
#include "chad/detail/lvr2.hpp"
#include "chad/detail/levels.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/submap.hpp"
#include "chad/detail/normals.hpp"

namespace chad::detail {
    void inline print_vec(glm::vec3 vec) {
        fmt::println("{:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(glm::aligned_vec3 vec) {
        fmt::println("{:.4f} {:.4f} {:.4f}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(glm::ivec3 vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(glm::aligned_ivec3 vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
}

namespace chad {
    TSDFMap::TSDFMap(float sdf_res, float sdf_trunc): _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
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
            if (glm::distance(position_vec, start) > 5.0f) {
                _active_submap_p->finalize(*_active_octree_p, *_node_levels_p, _sdf_trunc);
                _submaps.push_back(_active_submap_p);
                _active_submap_p = new Submap();
                _active_submap_p->positions.push_back(position_vec);
                _active_octree_p->clear();
            }
            // else just update active submap
            else positions.push_back(position_vec);
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
        fmt::println("total    {:.2f}", dur);
    }
    void TSDFMap::save(const std::string& filename) {
        // finalize current active submap
        if (!_active_submap_p->positions.empty()) {
            _active_submap_p->finalize(*_active_octree_p, *_node_levels_p, _sdf_trunc);
            _submaps.push_back(_active_submap_p);
        }

        // reconstruct 3D mesh using LVR2
        fmt::println("reconstructing the first submap");
        detail::reconstruct(*_submaps.front(), *_node_levels_p, _sdf_res, _sdf_trunc, filename);
    }

    // TSDFMap::iterator::iterator(const detail::NodeLevels& node_levels, uint32_t root_addr): _mc(0), _leaf_cluster_p(nullptr), _node_levels(node_levels) {
    //     _node_paths.fill(0);
    //     _node_addrs.fill(0);
    //     _node_addrs[0] = root_addr;
    // }
    // void inline TSDFMap::iterator::operator++() {
    //     // TODO
    // }
    // void inline TSDFMap::iterator::operator--() {
    //     // TODO
    // }
    // void inline TSDFMap::iterator::operator+(uint32_t increment) {
    //     (void)increment;
    //     // TODO
    // }
    // void inline TSDFMap::iterator::operator-(uint32_t decrement) {
    //     (void)decrement;
    //     // TODO
    // }
    // auto TSDFMap::begin(uint32_t root_addr) const -> iterator {
    //     iterator it = iterator(*_node_levels_p, root_addr);

    //     // validate root address
    //     if (root_addr >= _node_levels_p->_nodes[0]._raw_data.size()) {
    //         throw std::runtime_error("chad::TSDFMap::begin() given faulty root address");
    //         return it;
    //     }

    //     // walk to the first leaf node
    //     uint32_t depth = 0;
    //     while (true) {
    //         uint8_t child_i = it._node_paths[depth]++;

    //         // standard node
    //         if (depth < detail::NodeLevels::MAX_DEPTH - 1) { 
    //             // try to find the child in current node
    //             uint32_t node_addr = it._node_addrs[depth];
    //             uint32_t child_addr = it._node_levels.get_child_addr(depth, node_addr, child_i);

    //             // check if child address is valid
    //             if (child_addr > 0) {
    //                 // assign child addr at next depth
    //                 it._node_addrs[++depth] = child_addr;
    //             }
    //         }
    //         // leaf cluster node
    //         else {
    //             // try to get the leaf cluster, skip if it doesn't exist
    //             auto [cluster, cluster_exists] = it._node_levels.try_get_lc(it._node_addrs[depth], child_i);
    //             if (!cluster_exists) continue;
    //             // assign leaf cluster to iterator
    //             it._leaf_cluster_p = &cluster;
    //             break;
    //         }
    //     }

    //     // reconstruct morton code from path
    //     uint64_t code = 0;
    //     for (uint64_t k = 0; k < detail::NodeLevels::MAX_DEPTH; k++) {
    //         uint64_t part = it._node_paths[k] - 1;
    //         code |= part << uint64_t(60 - k*3);
    //     }
    //     it._mc = detail::MortonCode(code);
    //     return it;
    // }
    // auto TSDFMap::end(uint32_t root_addr) const -> iterator {
    //     iterator it = iterator(*_node_levels_p, root_addr);
    //     // walk to the last leaf node
    //     // TODO
    //     return it;
    // }
    // auto TSDFMap::cbegin(uint32_t root_addr) const -> const_iterator {
    //     return begin(root_addr);
    // }
    // auto TSDFMap::cend(uint32_t root_addr) const -> const_iterator {
    //     return begin(root_addr);
    // }
}